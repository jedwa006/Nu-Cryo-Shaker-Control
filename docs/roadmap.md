# Roadmap and immediate follow-ups

This plan captures the next set of firmware tasks after the PlatformIO migration and baseline health/Modbus bring-up.

## 1) Enable DIN (digital inputs) with isolation

- Wrap the Waveshare DIN helpers (`lib/waveshare_vendor/WS_DIN.*`) inside `lib/waveshare_hal/` and expose a clean API for reading all eight channels plus edge/level detection. The vendor examples use GPIOs 4–11 with `INPUT_PULLUP`; preserve the isolation inversion rules (`DIN_Inverse_Enable`). 
- Add a health component for DIN that reports presence and freshness (e.g., poll + stale timeout) and publishes `io/din/state` / `io/din/event` topics per the protocol scaffold.
- Wire critical inputs (e-stop/door) into the future run/estop state machine so a tripped DIN can immediately force `run_allowed=false` and `outputs_allowed=false`.

## 2) Enable RO (relay outputs) through the GPIO expander

- Integrate the TCA9554-based expander via a HAL wrapper (`lib/waveshare_vendor/WS_Relay.*` and `WS_TCA9554PWR.*`) so the application code can set relay bitmasks without vendor-specific headers. 
- Implement an `IoService` that owns relay state, enforces `outputs_allowed`, and handles MQTT commands (e.g., `io/cmd/event`) with acknowledgements consistent with `docs/protocol.md`.
- Add health reporting for the expander (probe + stale detection) and publish `io/dout/state` snapshots so HMI dashboards can render output status.

## 3) Finish run/estop control plane

- Build a run/hold/stop/estop state machine that consumes `HealthManager` signals and DIN events. Gate shaker motion and relay outputs on `run_allowed`/`outputs_allowed` to ensure priority ordering (estop/DIN > health faults > operator run commands). 
- Define MQTT command/ack topics for run control (start/stop/hold/reset) that align with the topic map in `docs/protocol.md`.

## 4) Align Modbus PID mapping and coverage

- Replace the placeholder PID registers (`REG_PV`, `REG_SV`, `REG_OUT_PCT`) with the real map and scaling for the deployed controllers; extend `PidModbusComponent` to surface alarms if available. 
- Add a periodic params publisher (`PID_PARAMS_PERIOD_MS`) once the register map is finalized so dashboards can visualize tuning/limits.

## 5) Validation and CI coverage

- Add unit-style component tests (where feasible) or host-side fakes to cover health aggregation, stale detection, and MQTT topic formation.
- Extend GitHub Actions to build any new PlatformIO environments added for IO testing (e.g., a mock DIN/RO build that stubs hardware drivers) and publish artifacts for bench flashing.
