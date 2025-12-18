# Roadmap and immediate follow-ups

This plan captures the next set of firmware tasks after the PlatformIO migration and baseline health/Modbus bring-up.

## 1) DIN (digital inputs) with isolation

- ✅ Implemented: `waveshare_hal::DinHal` wraps `WS_DIN.*` with inversion preserved, `DinComponent` (required) drives health, and `io/din/state` + `io/din/event` publish the 8-bit mask (estop/lid/door on bits 0–2) at `IO_STATE_PERIOD_MS`.
- Next: feed DIN interlocks into the run/estop state machine once it lands, and add any bit-to-signal mapping needed for auxiliary sensors.

## 2) RO (relay outputs) through the GPIO expander

- ✅ Implemented: `waveshare_hal::RelayHal` wraps the TCA9554 expander, `RelayComponent` tracks health/state, and `IoService` publishes `io/dout/state` while handling `io/cmd/event` with `io/dout/ack` responses. Commands are blocked when `outputs_allowed=false`.
- Next: add default power-on output policies, per-channel safety overrides, and broaden ack payloads if the dashboard needs more context.

## 3) Finish run/estop control plane

- Build a run/hold/stop/estop state machine that consumes `HealthManager` signals and DIN events. Gate shaker motion and relay outputs on `run_allowed`/`outputs_allowed` to ensure priority ordering (estop/DIN > health faults > operator run commands). 
- Define MQTT command/ack topics for run control (start/stop/hold/reset) that align with the topic map in `docs/protocol.md`.

## 4) Align Modbus PID mapping and coverage

- Replace the placeholder PID registers (`REG_PV`, `REG_SV`, `REG_OUT_PCT`) with the real map and scaling for the deployed controllers; extend `PidModbusComponent` to surface alarms if available. 
- Add a periodic params publisher (`PID_PARAMS_PERIOD_MS`) once the register map is finalized so dashboards can visualize tuning/limits.

## 5) Validation and CI coverage

- Add unit-style component tests (where feasible) or host-side fakes to cover health aggregation, stale detection, and MQTT topic formation.
- Extend GitHub Actions to build any new PlatformIO environments added for IO testing (e.g., a mock DIN/RO build that stubs hardware drivers) and publish artifacts for bench flashing.
