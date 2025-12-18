# Monitoring MQTT Status with Node-RED

This guide covers how to check firmware MQTT behavior using only a PC (Linux or macOS) and a Node-RED dashboard. It assumes you have an MQTT broker you can reach from your machine (e.g., Mosquitto) and the firmware publishing to topics under its configured root.

## Prerequisites
- Node.js 18+ installed on your PC
- npm available in PATH
- Access credentials/host/port for the MQTT broker the device uses
- Basic familiarity with the firmware topics (defaults are under `<root>/` as used by `MqttBus`)

## Setup Node-RED locally
1. Install Node-RED globally:
   ```bash
   npm install -g --unsafe-perm node-red
   ```
2. Start Node-RED:
   ```bash
   node-red
   ```
   - Default UI: http://127.0.0.1:1880
   - Default dashboard (after installing `node-red-dashboard`): http://127.0.0.1:1880/ui

## Create a flow to monitor firmware topics
1. Open the Node-RED editor at http://127.0.0.1:1880.
2. Install dashboard nodes (once per machine):
   - Menu → Manage palette → Install → search `node-red-dashboard` → Install.
3. Drag the following nodes onto the canvas:
   - **mqtt in** (one per topic you want to watch)
   - **json** (to parse payloads)
   - **ui_text** or **ui_chart** (for display)
   - **debug** (optional, to view raw messages)
4. Configure the MQTT broker in the first `mqtt in` node:
   - Server: your broker hostname/IP
   - Port: usually 1883 (or 8883 with TLS)
   - Username/password if required
   - Topic wildcards to capture the firmware publishes, e.g.:
     - `<root>/sys/heartbeat`
     - `<root>/health/+/state`
     - `<root>/io/din/state`
     - `<root>/io/dout/state`
     - `<root>/io/din/event`
5. Wire `mqtt in` → `json` → `debug` to confirm messages arrive.
6. Add dashboard widgets:
   - Heartbeat: `mqtt in` (topic `<root>/sys/heartbeat`) → `json` → `ui_text` (Value format: `{{payload.uptime_s}} s`, label “Uptime”).
   - System health: `mqtt in` (topic `<root>/health/system`) → `json` → `ui_text` (Value `{{payload.system_state}}`, label “System state”).
   - DIN state: `mqtt in` (topic `<root>/io/din/state`) → `json` → `ui_text` (Value `{{payload.mask}}`, label “DIN mask”).
   - Relay outputs: `mqtt in` (topic `<root>/io/dout/state`) → `json` → `ui_text` (Value `{{payload.mask}}`, label “Relay mask”).
   - DIN events: `mqtt in` (topic `<root>/io/din/event`) → `json` → `debug` (shows rising/falling edges and masks).
7. Deploy the flow (top right “Deploy” button).

## Optional: trigger IO commands from Node-RED
- Add **mqtt out** nodes to publish to `<root>/io/cmd/event` with JSON payloads like:
  ```json
  { "cmd_id": 1, "mask": 0x03 }
  ```
- Use an **inject** node to send preset JSON strings for quick tests (e.g., toggle relays or set a mask).

## What to watch
- Heartbeat updates every few seconds with `uptime_s`.
- Health topics show `OK/DEGRADED/FAULT` and counts; use this to confirm interlocks or sensors.
- DIN state/event topics show live changes from estop/door/lid inputs.
- DOUT state shows relay mask and whether outputs are allowed.

## Capturing evidence
- Use the debug sidebar to capture payloads when testing changes.
- Export the flow (Menu → Export → Clipboard) to share a reproducible monitoring setup with the team.
