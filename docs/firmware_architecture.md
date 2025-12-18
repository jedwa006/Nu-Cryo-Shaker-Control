# Firmware architecture and data flow

This document summarizes how the PlatformIO firmware (`firmware_esp32s3_pio/nu_cryo_control`) is wired together, which layers own each concern, and how the health / run / output “levels” interact.

## System view

- **Hardware:** Waveshare ESP32-S3-ETH-8DI-8RO with W5500 Ethernet, isolated DIN, relay outputs, and RS-485 for PID controllers.
- **Transport:** Ethernet → MQTT broker (Pi) and RS-485 Modbus RTU for the PIDs.
- **Publish/Subscribe root:** `<MACHINE_ID>/<NODE_ID>/` (see `include/app/app_config.h`).

## Runtime layers

The runtime is composed of a few small services that are stitched together in `src/main.cpp`:

1. **Network bring-up (`AppNetworkManager`)** – Resets and configures the W5500, listens for link/IP events, and exposes connection status plus `NetworkClient` for MQTT. Static addressing is applied when a non-zero IP is configured; otherwise DHCP is used. The manager also asserts a hardware reset pulse on boot to avoid W5500 init flakiness. 

2. **MQTT bus (`MqttBus`)** – Stores the `<MACHINE_ID>/<NODE_ID>` root, registers a single callback shim, and provides helpers to publish JSON payloads to subtopics (e.g., `sys/heartbeat`, `status/lwt`). Connection retries and server selection live in `main.cpp`. 

3. **Health framework (`HealthManager` + components)** – Each component implements `IHealthComponent` to expose status, freshness, and expectations. Components register via `HealthRegistry`, which marks them as **expected** and optionally **required**. The manager aggregates reports, applies stale timeouts, and derives `run_allowed` / `outputs_allowed`. Required components in `MISSING/ERROR/STALE` force a FAULT-like system state; optional failures only degrade the system. 
   - **Implemented components:** Ethernet link (`EthHealthComponent`), DIN interlocks (`DinComponent`), relay expander (`RelayComponent`), and three Modbus PIDs (`PidModbusComponent` via `FieldbusService`). The DIN + relay components are marked required so estop/door/lid interlocks and output hardware faults force `run_allowed=false` / `outputs_allowed=false`. An optional ADXL345 accelerometer component exists for future vibration monitoring. 

4. **Fieldbus service (`FieldbusService`)** – Owns the Modbus RTU master (`HardwareSerial` + `ModbusRTU`), instantiates PID components, and ticks them at `PID_STATE_PERIOD_MS` (~5 Hz). Register offsets for PV/SV/output are placeholders in `app_config.h` and should be aligned with your PID controllers. 

5. **IO service (`IoService` + HAL wrappers)** – Wraps the vendor DIN (`WS_DIN.*`) and relay expander (`WS_Relay.*` / `WS_TCA9554PWR.*`) behind `waveshare_hal/`. Ticks DIN/relay at `IO_STATE_PERIOD_MS` (~5 Hz), publishes `io/din/state` + `io/din/event`, publishes `io/dout/state`, and handles `io/cmd/event` commands with `io/dout/ack` responses. Output commands are rejected when `outputs_allowed=false`. DIN uses bits 0–2 for estop, lid, and door interlocks (inversion enabled).

6. **Publishers** – `publish_heartbeat`, `publish_system_health`, `publish_component_health`, PID publishers, and the IO publishers live here. Heartbeat/health cadence is controlled in `main.cpp` using the constants in `app_config.h`. 

## Loop and data flow

1. **Setup** – Initialize the MQTT bus root, register health components, start the fieldbus, then bring up Ethernet. 
2. **Loop** –
   - Tick Ethernet health (~4 Hz) and Modbus PIDs (~5 Hz) to refresh component status. 
   - Tick DIN/relay IO (~5 Hz), publish DIN edges/state, and publish relay state. IO commands are dispatched through the MQTT bus handler.
   - When Ethernet is up, attempt MQTT connects (with a retained offline LWT). On success, publish boot metadata and mark the node online. 
   - Evaluate aggregate health (including stale detection and DIN interlocks), pump MQTT callbacks, and publish heartbeat/system/component health at their configured intervals. 

## Levels and priorities

- **Health level:** Aggregates component health into `system_state`, `degraded`, `run_allowed`, and `outputs_allowed`. Required component failures force `run_allowed=false` and `outputs_allowed=false`, which should gate any run/estop logic. Optional failures keep `run_allowed=true` but mark the system degraded. 
- **Run / estop level:** A full run/hold/stop/estop state machine is not yet implemented in this branch. When adding it, consume the health-derived flags above so that emergency inputs (e.g., DIN estop/door/lid via `DinComponent`) and Modbus faults can immediately inhibit motion or outputs. Relay output commands are already gated on `outputs_allowed` in the IO service.

## Tricky or easy-to-miss details

- **Ethernet reliability:** A hardware reset pulse is issued to the W5500 before `ETH.begin` to avoid occasional reset timeouts, and static IPs are only configured when all octets are non-zero; otherwise DHCP is used. 
- **Stale handling:** Components can define a `stale_timeout_ms`; the manager will treat a previously-OK component as `STALE` if it stops reporting fresh data (e.g., Modbus read failures beyond 1.5 s). 
- **PID register map:** `REG_PV`, `REG_SV`, and `REG_OUT_PCT` are placeholders; align them with your controller’s scaling and extend the Modbus driver accordingly. 
- **IO expansion hooks:** Vendor examples for DIN and relay outputs (TCA9554 GPIO expander, opto-isolated inputs) live under `lib/waveshare_vendor/`. These are wrapped in `lib/waveshare_hal/` (`DinHal`, `RelayHal`) so the application layer stays vendor-agnostic while still honoring the board’s inversion rules and pin mappings.
