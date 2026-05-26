"""
River Horizon — Meshtastic Backup Communication Beacon

Provides an off-grid backup channel via LoRa mesh radio (Meshtastic).
When 4G LTE drops mid-flight, this is the only way to find the drone
or trigger RTH remotely.

Hardware
--------
One Meshtastic-compatible LoRa node connected via USB serial to the Pi.
Tested with: Heltec LoRa32 V3, LILYGO T-Beam, RAK WisBlock 4631.
Typical range: 1–5 km line of sight in open sky.

Packet format  (pipe-delimited — kept tiny to respect LoRa duty-cycle)
----------------------------------------------------------------------
Outbound beacon:
    RH|<drone_id>|<lat>|<lng>|<alt_m>|<bat%>|<mode>
    e.g.  RH|RH-ALPHA-01|40.71298|-74.00618|35.0|85|GUIDED

Inbound commands:
    KILL <drone_id>    → immediate LAND (motors cut after touchdown)
    RTH  <drone_id>    → Return to Home
    WHERE <drone_id>   → immediate beacon broadcast
    STATUS <drone_id>  → extended status reply

Beacon cadence
--------------
- In flight (airborne): every 30 s
- On ground:            every 60 s
- Immediately on receiving WHERE or KILL command
- Immediately on any fault/e-stop event (call request_beacon())

Falls back to sim/log-only mode when meshtastic package or hardware
is not available — the main stack continues normally.
"""

import asyncio
import logging
import time
from typing import Any, Callable, Coroutine, Optional

logger = logging.getLogger(__name__)

BEACON_INTERVAL_FLIGHT_S: float = 30.0
BEACON_INTERVAL_GROUND_S: float = 60.0
LOW_SIGNAL_THRESHOLD: int = 20   # RSSI %

DEFAULT_PORT: str = "/dev/ttyUSB1"


class MeshtasticBeacon:
    """
    Async Meshtastic LoRa backup beacon for River Horizon.

    Integrates with the MAVLink bridge (for position/battery/mode reads)
    and the fault manager (for RTH/LAND dispatch on incoming commands).

    Args:
        drone_id:         Drone unit identifier (e.g. 'RH-ALPHA-01').
        port:             Serial port of the Meshtastic node.
        bridge:           MAVLinkBridge for vehicle state reads.
        fault_manager:    FaultManager for RTH/LAND dispatch.
        on_rth:           Coroutine callable for RTH (e.g. rth_controller.execute_rth).
        on_land:          Coroutine callable for emergency land.
        cellular_rssi:    Callable returning current RSSI dBm or None.
        sim_mode:         Force sim/log-only mode.
    """

    def __init__(
        self,
        drone_id: str,
        port: str = DEFAULT_PORT,
        bridge=None,
        fault_manager=None,
        on_rth: Optional[Callable[..., Coroutine]] = None,
        on_land: Optional[Callable[..., Coroutine]] = None,
        cellular_rssi: Optional[Callable[[], Optional[int]]] = None,
        sim_mode: bool = False,
    ) -> None:
        self._drone_id = drone_id
        self._port = port
        self._bridge = bridge
        self._fault_manager = fault_manager
        self._on_rth = on_rth
        self._on_land = on_land
        self._cellular_rssi = cellular_rssi
        self._sim = sim_mode

        self._iface: Any = None
        self._running = False
        self._force_beacon = asyncio.Event()
        self._command_queue: asyncio.Queue = asyncio.Queue()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    async def initialise(self) -> None:
        """
        Connect to the Meshtastic node. Falls back to sim mode on failure.
        Must be called before run().
        """
        if not self._sim:
            connected = await asyncio.get_event_loop().run_in_executor(
                None, self._connect_sync
            )
            if not connected:
                self._sim = True

        logger.info(
            "MeshtasticBeacon initialised for '%s' (sim=%s, port=%s).",
            self._drone_id, self._sim, self._port,
        )

    async def run(self) -> None:
        """
        Main async loop. Runs until cancelled.
        Sends periodic beacons and dispatches received commands.
        """
        self._running = True
        try:
            await asyncio.gather(
                self._beacon_loop(),
                self._command_dispatch_loop(),
            )
        except asyncio.CancelledError:
            pass
        finally:
            self._running = False

    async def shutdown(self) -> None:
        """Stop the beacon and close the Meshtastic connection."""
        self._running = False
        self._force_beacon.set()
        await asyncio.get_event_loop().run_in_executor(None, self._disconnect_sync)
        logger.info("MeshtasticBeacon shut down.")

    def request_beacon(self) -> None:
        """Trigger an immediate beacon (e.g. on fault or e-stop)."""
        self._force_beacon.set()

    # ------------------------------------------------------------------
    # Beacon loop
    # ------------------------------------------------------------------

    async def _beacon_loop(self) -> None:
        """Periodic position broadcast."""
        while self._running:
            await self._send_beacon()

            # Adaptive cadence: shorter in flight, shorter on weak signal
            airborne = await self._is_airborne()
            interval = (
                BEACON_INTERVAL_FLIGHT_S if airborne else BEACON_INTERVAL_GROUND_S
            )

            try:
                await asyncio.wait_for(self._force_beacon.wait(), timeout=interval)
                self._force_beacon.clear()
            except asyncio.TimeoutError:
                pass

    async def _send_beacon(self) -> None:
        """Build and transmit a compact position packet."""
        snap = await self._get_snapshot()

        lat   = snap.get("latitude",  0.0)
        lng   = snap.get("longitude", 0.0)
        alt   = snap.get("relative_altitude_m", 0.0)
        bat   = snap.get("battery_pct", 0.0)
        mode  = snap.get("flight_mode", "UNKNOWN")

        packet = (
            f"RH|{self._drone_id}|"
            f"{lat:.5f}|{lng:.5f}|"
            f"{alt:.1f}|{bat:.0f}|{mode}"
        )

        if self._sim:
            logger.info("Meshtastic [SIM] TX: %s", packet)
            return

        await asyncio.get_event_loop().run_in_executor(
            None, self._send_text_sync, packet
        )

    # ------------------------------------------------------------------
    # Command dispatch loop
    # ------------------------------------------------------------------

    async def _command_dispatch_loop(self) -> None:
        """Dequeue and process commands that arrived on the mesh."""
        while self._running:
            try:
                command = await asyncio.wait_for(
                    self._command_queue.get(), timeout=1.0
                )
                await self._handle_command(command)
            except asyncio.TimeoutError:
                pass
            except asyncio.CancelledError:
                break

    async def _handle_command(self, command: str) -> None:
        """Dispatch a validated command string."""
        parts = command.split()
        verb   = parts[0].upper() if parts else ""
        target = parts[1] if len(parts) > 1 else ""

        if target != self._drone_id:
            return

        logger.warning(
            "Meshtastic command received: '%s' for '%s'", verb, self._drone_id
        )

        if verb == "KILL":
            logger.critical("Meshtastic KILL — initiating emergency LAND.")
            if self._on_land:
                await self._on_land()
            self._force_beacon.set()

        elif verb == "RTH":
            logger.warning("Meshtastic RTH — returning to home.")
            if self._on_rth:
                from core.constants import FaultReason
                await self._on_rth(FaultReason.RIVER_SONG_COMMAND)
            self._force_beacon.set()

        elif verb == "WHERE":
            logger.info("Meshtastic WHERE — sending immediate beacon.")
            self._force_beacon.set()

        elif verb == "STATUS":
            await self._send_status_reply()

    async def _send_status_reply(self) -> None:
        """Send a verbose status reply to the mesh."""
        snap = await self._get_snapshot()
        lat  = snap.get("latitude",  0.0)
        lng  = snap.get("longitude", 0.0)
        alt  = snap.get("relative_altitude_m", 0.0)
        bat  = snap.get("battery_pct", 0.0)
        mode = snap.get("flight_mode", "UNKNOWN")
        armed = snap.get("armed", False)

        reply = (
            f"STATUS|{self._drone_id}|"
            f"pos={lat:.5f},{lng:.5f}|"
            f"alt={alt:.1f}m|"
            f"bat={bat:.0f}%|"
            f"mode={mode}|"
            f"armed={armed}"
        )

        if self._sim:
            logger.info("Meshtastic [SIM] STATUS reply: %s", reply)
            return

        await asyncio.get_event_loop().run_in_executor(
            None, self._send_text_sync, reply
        )

    # ------------------------------------------------------------------
    # Vehicle state helpers
    # ------------------------------------------------------------------

    async def _get_snapshot(self) -> dict:
        """Return a vehicle state snapshot from the MAVLink bridge."""
        if self._bridge is None:
            return {}
        try:
            return await self._bridge.vehicle_state.snapshot()
        except Exception:
            return {}

    async def _is_airborne(self) -> bool:
        """True if the vehicle is in a flight mode and has altitude."""
        snap = await self._get_snapshot()
        return snap.get("relative_altitude_m", 0.0) > 0.5

    # ------------------------------------------------------------------
    # Sync helpers (run via executor)
    # ------------------------------------------------------------------

    def _connect_sync(self) -> bool:
        """Open the Meshtastic serial interface (blocking)."""
        try:
            import meshtastic.serial_interface
            from pubsub import pub

            self._iface = meshtastic.serial_interface.SerialInterface(self._port)
            pub.subscribe(self._on_receive_sync, "meshtastic.receive.text")
            logger.info("Meshtastic node connected on %s.", self._port)
            return True
        except Exception as exc:
            logger.warning(
                "Meshtastic hardware not found on %s (%s) — sim mode.",
                self._port, exc,
            )
            return False

    def _disconnect_sync(self) -> None:
        if self._iface:
            try:
                self._iface.close()
            except Exception:
                pass
            self._iface = None

    def _send_text_sync(self, text: str) -> None:
        """Send a text message (blocking — call via executor)."""
        try:
            self._iface.sendText(text, destinationId="^all")
            logger.debug("Meshtastic TX: %s", text)
        except Exception as exc:
            logger.error("Meshtastic send error: %s", exc)

    def _on_receive_sync(self, packet, interface=None) -> None:
        """
        Meshtastic receive callback (fires on pubsub thread).
        Puts the command text onto the async queue for safe dispatch.
        """
        try:
            decoded = packet.get("decoded", {})
            text: str = decoded.get("text", "").strip()
            if text:
                # Thread-safe enqueue into the async queue
                loop = asyncio.get_event_loop()
                if loop.is_running():
                    loop.call_soon_threadsafe(self._command_queue.put_nowait, text)
        except Exception as exc:
            logger.error("Meshtastic receive handler error: %s", exc)

    def __repr__(self) -> str:
        return (
            f"MeshtasticBeacon(drone={self._drone_id!r}, "
            f"port={self._port!r}, sim={self._sim})"
        )
