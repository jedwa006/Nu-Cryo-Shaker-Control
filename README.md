# Nu Cryo Shaker Control

Control stack for the cryogenic ball mill / shaker system. The repo currently hosts:

- **PlatformIO firmware** for the Waveshare ESP32-S3-ETH-8DI-8RO board (`firmware_esp32s3_pio/nu_cryo_control`).
- **Node-RED / FlowFuse UI** assets (`ui_nodered`).
- **Arduino-style legacy firmware scaffold** (kept for reference under `firmware_esp32s3_arduino`).

## Quick start

1. Install [PlatformIO](https://platformio.org/) (VS Code extension or CLI).
2. Open `firmware_esp32s3_pio/nu_cryo_control` in PlatformIO.
3. Adjust `include/app/app_config.h` for your deployment (broker IP/port, static IP, Modbus IDs and register offsets).
4. Build the firmware:
   ```bash
   cd firmware_esp32s3_pio/nu_cryo_control
   pio run -e esp32s3_arduino
   ```

GitHub Actions runs the same build on PRs via `.github/workflows/firmware-ci.yml` and publishes the resulting ELF/BIN artifacts.

## Repository map

- `firmware_esp32s3_pio/nu_cryo_control/` – PlatformIO project (W5500 Ethernet, Modbus RTU PIDs, health publishing). See `README_INTEGRATION.md` inside for migration notes.
- `docs/` – Project-level docs (network defaults, MQTT protocol, architecture, roadmap).
- `ui_nodered/` – Dashboard flows (kept alongside firmware for now).
- `.github/workflows/firmware-ci.yml` – CI that builds `esp32s3_arduino` environment on pushes/PRs.

## Current firmware behavior (high level)

- Brings up the W5500 Ethernet interface with optional static IP fallback to DHCP. Publishes boot metadata, a retained LWT (`status/lwt`), and per-component/system health snapshots under `<MACHINE_ID>/<NODE_ID>/...`.
- Polls three Modbus RTU PID controllers (~5 Hz) and reports PV/SV/output% along with health state. Register offsets are placeholders pending alignment with the deployed PIDs.
- Uses `RunControl` to combine health + DIN interlocks into a run/hold/stop/estop state machine, exposing `run_allowed` / `outputs_allowed` gating for IO and motion control.

For deeper details on MQTT topics, message cadence, and the health/run levels, see `docs/firmware_architecture.md` and `docs/protocol.md`.

## Next steps

See `docs/roadmap.md` for the prioritized follow-up work, including enabling the isolated digital inputs (DIN) and relay outputs (RO) using the GPIO expander path.
