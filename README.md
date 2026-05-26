# River Horizon

Autonomous and remote-controlled drone fleet management system for the **River Song AI ecosystem** ([riversongai.com](https://riversongai.com)).

River Song is the core AI brain. River Horizon controls a fleet of drones capable of fully autonomous waypoint-based flight, manual web control, live video streaming, and 4G LTE telemetry — all integrated with River Song voice commands and scheduling.

---

## Capabilities

- Fully autonomous waypoint-based flight (MAVLink / ArduPilot / BetaFlight)
- Manual remote control via web interface (FastAPI + WireGuard VPN)
- Live MJPEG video streaming — multiple drones simultaneously, on-demand
- Fleet management — multiple units from a single River Song dashboard
- River Song voice command integration: *"River, launch Horizon unit 1 to waypoint alpha"*
- 4G LTE telemetry and control via cellular interface
- Return to Home on low battery, signal loss, or geofence breach
- Geofence enforced at all times — drone cannot leave the defined boundary

---

## Onboard Stack

| Component | Hardware |
|---|---|
| Flight Controller | Pixhawk 6C or BetaFlight FC |
| Companion Computer | Raspberry Pi Zero 2W or Pi 5 |
| Connectivity | 4G LTE cellular + WireGuard VPN |
| Protocol | MAVLink 2 (pymavlink) |
| Vision | OpenCV (headless) |
| API | FastAPI + uvicorn |

---

## Directory Structure

```
river-horizon/
├── core/
│   ├── main.py           # Entry point — bootstraps all subsystems
│   ├── config.py         # Typed config loader (drone_profile.json + env vars)
│   └── constants.py      # All system-wide constants and enumerations
├── flight/
│   ├── mavlink_bridge.py # Async MAVLink connection and message dispatch
│   ├── flight_controller.py  # Arm/disarm, takeoff, land, goto
│   ├── mode_manager.py   # Flight mode transitions with safety validation
│   ├── waypoint_manager.py   # Named waypoints and mission execution
│   └── return_home.py    # RTH sequence controller
├── safety/
│   ├── geofence.py       # Cylindrical geofence enforcer (always active)
│   ├── battery_monitor.py    # Low/critical battery failsafe
│   ├── signal_watchdog.py    # MAVLink + cellular signal loss detection
│   └── fault_manager.py  # Central fault aggregator and failsafe dispatcher
├── vision/
│   ├── camera_manager.py # OpenCV VideoCapture abstraction
│   ├── stream_server.py  # MJPEG-over-HTTP stream server (aiohttp)
│   └── obstacle_detect.py    # Background subtraction obstacle detection
├── telemetry/
│   ├── collector.py      # Periodic telemetry collection and distribution
│   ├── logger.py         # Rotating JSONL telemetry log writer
│   └── alerts.py         # Threshold-based alert manager
├── connectivity/
│   ├── cellular.py       # 4G LTE signal quality monitor
│   ├── vpn.py            # WireGuard VPN lifecycle manager
│   ├── api_client.py     # River Song API HTTP client (retry + backoff)
│   └── stream_manager.py # On-demand video stream session manager
├── remote/
│   ├── web_controller.py # FastAPI REST API (/api/horizon/)
│   └── input_handler.py  # Command parser and validator
├── units/
│   └── drone_profile.json    # Per-unit hardware and safety configuration
└── tests/
    ├── test_flight.py
    ├── test_safety.py
    └── test_telemetry.py
```

---

## Getting Started

### 1. Install dependencies

```bash
pip install -r requirements.txt
```

### 2. Configure the drone profile

Edit `units/drone_profile.json` with your drone's actual coordinates, MAVLink connection string, and safety limits.

### 3. Set environment variables

```bash
export RIVER_SONG_API_URL="https://api.riversongai.com"
export RIVER_SONG_API_KEY="your-api-key-here"
export DRONE_PROFILE="/path/to/units/drone_profile.json"   # optional
export WEB_CONTROLLER_PORT=8080                             # optional
```

Never store API keys in `drone_profile.json`. Use environment variables or a `.env` file.

### 4. Run

```bash
python -m core.main
```

---

## API Endpoints

All endpoints are served under `/api/horizon/` and accessible via WireGuard VPN.

| Method | Path | Description |
|---|---|---|
| GET | `/api/horizon/status` | Drone status + telemetry snapshot |
| GET | `/api/horizon/telemetry` | Latest telemetry packet |
| POST | `/api/horizon/command` | Execute a flight command |
| GET | `/api/horizon/waypoints` | List registered named waypoints |
| POST | `/api/horizon/stream/start` | Start video stream for this client |
| POST | `/api/horizon/stream/stop` | Stop video stream for this client |
| POST | `/api/horizon/stream/keepalive` | Keep stream session alive |
| GET | `/api/horizon/health` | Health check |

### Command payload format

```json
{
  "command": "goto",
  "params": {
    "latitude": 51.5074,
    "longitude": -0.1278,
    "altitude_m": 30.0
  }
}
```

Supported commands: `arm`, `disarm`, `takeoff`, `land`, `rth`, `goto`, `goto_waypoint`, `set_velocity`, `start_mission`, `pause_mission`, `resume_mission`, `abort_mission`, `start_stream`, `stop_stream`, `set_mode`, `reload_config`.

Write commands require the `X-River-Song-Key` header.

---

## Video Stream

The MJPEG stream is served at:

```
http://<vpn-ip>:8554/stream
```

Embed in any browser with a standard `<img>` tag:

```html
<img src="http://<vpn-ip>:8554/stream" />
```

Streams are **off by default** to conserve 4G LTE bandwidth. They activate on-demand when a client calls `/api/horizon/stream/start`.

---

## Safety Architecture

Safety systems initialise **before** all other subsystems. The boot order is:

1. FaultManager
2. GeofenceMonitor → BatteryMonitor → SignalWatchdog
3. MAVLink bridge + flight systems
4. Connectivity (cellular, VPN, API client)
5. Vision (camera, stream server, obstacle detector)
6. Telemetry
7. Web controller

Any fault reported to the FaultManager triggers the appropriate failsafe:

| Fault | Response |
|---|---|
| Low battery (≤20%) | Return to Home |
| Critical battery (≤10%) | Immediate Land |
| Signal loss (>5s) | Return to Home |
| Geofence breach | Return to Home |
| Hardware fault | Immediate Land |

The geofence **cannot be disabled at runtime**.

---

## River Song Voice Commands

River Horizon responds to River Song voice commands via the `/api/horizon/command` endpoint:

- *"River, launch Horizon unit 1 to waypoint alpha"* → `goto_waypoint` command
- *"River, bring Horizon unit 1 home"* → `rth` command
- *"River, show me the Horizon unit 1 feed"* → `start_stream` command
- *"River, land Horizon unit 1"* → `land` command

---

## Running Tests

```bash
pytest tests/ -v
```

---

## Fleet Management

Each drone unit has its own `drone_profile.json`. To manage multiple units, run one River Horizon instance per drone (each on its own companion computer), all connecting back to the same River Song dashboard via WireGuard VPN.

---

## License

Part of the River Song AI Ecosystem. All rights reserved.
