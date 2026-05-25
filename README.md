# River Horizon

River Horizon is a sophisticated drone management and flight control system designed for autonomous operations over 4G LTE/5G cellular networks. It features robust MAVLink integration, geofencing, battery monitoring, and automated Return-To-Home (RTH) logic.

## Project Structure

- `core/`: Main entry point, configuration, and constants.
- `flight/`: MAVLink bridge, flight controllers, and mission management.
- `safety/`: Geofence, battery monitoring, and fail-safe logic.
- `vision/`: Camera streams and obstacle detection.
- `telemetry/`: Data collection, logging, and alerting system.
- `connectivity/`: Cellular, VPN, and API client management.
- `remote/`: Web-based control interface and input handling.
- `units/`: Hardware-specific drone profiles.
- `tests/`: Automated test suite.

## Key Features

- **Cellular Command & Control:** Optimized for high-latency, low-bandwidth 4G/5G links.
- **Advanced RTH:** Intelligent Return-To-Home logic based on battery state and signal quality.
- **Geofence Protection:** Multi-layered geofencing for safe operations.
- **Vision-Assisted Flight:** Integrated obstacle detection and live streaming.
- **Telemetry Analytics:** Comprehensive logging and real-time alerts.

## Getting Started

1. Install dependencies: `pip install -r requirements.txt`
2. Configure your drone profile in `units/drone_profile.json`.
3. Run the main application: `python core/main.py`
