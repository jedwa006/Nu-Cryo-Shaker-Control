# Roadmap and immediate follow-ups

This plan captures the next set of firmware tasks after the PlatformIO migration and baseline health/Modbus bring-up.

## 1) DIN (digital inputs) with isolation

- ✅ Implemented: `waveshare_hal::DinHal` wraps `WS_DIN.*` with inversion preserved, `DinComponent` (required) drives health, and `io/din/state` + `io/din/event` publish the 8-bit mask (estop/lid/door on bits 0–2) at `IO_STATE_PERIOD_MS`.
- Next: add any bit-to-signal mapping needed for auxiliary sensors and confirm the expected wiring against the interlock definitions.

## 2) RO (relay outputs) through the GPIO expander

- ✅ Implemented: `waveshare_hal::RelayHal` wraps the TCA9554 expander, `RelayComponent` tracks health/state, and `IoService` publishes `io/dout/state` while handling `io/cmd/event` with `io/dout/ack` responses. Commands are blocked when `outputs_allowed=false`.
- Next: add default power-on output policies, per-channel safety overrides, and broaden ack payloads if the dashboard needs more context.

## 3) Finish run/estop control plane

- ✅ Implemented: `RunControl` owns a run/hold/stop/estop state machine, consumes `HealthManager` + DIN interlocks, publishes status, and gates `run_allowed`/`outputs_allowed` for IO command enforcement.
- Next: validate `run/cmd` + `run/ack` flows with the HMI dashboard and document any operator UI expectations.
- Next: tune state transitions/reasons with real hardware inputs, and run integration testing of the MQTT run command/ack flow (start/stop/hold/reset) alongside DIN/health faults.

## 4) Align Modbus PID mapping and coverage

- Replace the placeholder PID registers (`REG_PV`, `REG_SV`, `REG_OUT_PCT`) with the real map and scaling for the deployed controllers; extend `PidModbusComponent` to surface alarms if available. 
- ✅ Implemented: periodic params publisher (`PID_PARAMS_PERIOD_MS`) to support tuning/limits visualization.
- Next: finalize the register map + scaling and verify params against a known-good controller.

## 5) Validation and CI coverage

- Add unit-style component tests (where feasible) or host-side fakes to cover health aggregation, stale detection, and MQTT topic formation.
- Extend GitHub Actions to build any new PlatformIO environments added for IO testing (e.g., a mock DIN/RO build that stubs hardware drivers) and publish artifacts for bench flashing.
