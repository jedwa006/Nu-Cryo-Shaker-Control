# Cryo mill MQTT protocol

This document tracks the MQTT topics and state machine used by the cryo shaker control stack. It covers two layers:

- **Current PlatformIO baseline (health + PID telemetry):** Implemented today in `firmware_esp32s3_pio/nu_cryo_control`, rooted at `<MACHINE_ID>/<NODE_ID>/` with heartbeat/health/PID topics.
- **Run/HMI control draft (v0):** The legacy `mill/*` topic map below describes the original HMI command set. The run/estop control plane is now implemented under the `<MACHINE_ID>/<NODE_ID>/` root, so treat the `mill/*` map as historical reference.

---

## Current PlatformIO baseline topics

Root prefix: `<MACHINE_ID>/<NODE_ID>/` (e.g., `cryo_mill_01/esp32a`). Topic names come from `include/config/board_waveshare.h`.

| Direction | Topic                           | Notes |
|-----------|---------------------------------|-------|
| MCU → HMI | `status/lwt` (retained, QoS1)   | Last Will: `state: offline`; publishes `state: online` on connect. |
| MCU → HMI | `status/boot` (retained)        | Boot metadata: schema version, timestamp, node id, firmware name, Ethernet status/IP. |
| MCU → HMI | `sys/heartbeat` (QoS0)          | 1 Hz heartbeat with uptime and timestamps. |
| MCU → HMI | `status/health` (QoS0)          | Aggregate health: `system_state` (`OK/DEGRADED/FAULT`), run state (`run_state`, `run_reason`), inhibit flags (`run_allowed`, `outputs_allowed`), warning/critical counts. |
| MCU → HMI | `health/<component>/state`      | Per-component health snapshots (Ethernet, PID controllers, optional sensors). |
| MCU → HMI | `pid/<name>/state` (QoS0)       | PV/SV/output% + validity at ~5 Hz for each configured PID (`pid_heat1`, `pid_heat2`, `pid_cool1`). |
| MCU → HMI | `io/din/state` (QoS0)           | Digital input mask (8 bits, DIN inversion applied) at `IO_STATE_PERIOD_MS` (200 ms). |
| MCU → HMI | `io/din/event` (QoS0)           | Emitted on DIN edge/change with `rising/falling` masks and `prev_mask`. |
| HMI → MCU | `io/cmd/event` (QoS0/1)         | IO command input: set full relay mask (`mask`) or a single `channel` + `state`. Includes `cmd_id`. |
| MCU → HMI | `io/dout/state` (QoS0)          | Relay output mask + `outputs_allowed` at `IO_STATE_PERIOD_MS` (200 ms). |
| MCU → HMI | `io/dout/ack` (QoS0/1)          | Ack for `io/cmd/event` with `cmd_id/ok/err` + resulting mask/permissions. |
| HMI → MCU | `run/cmd` (QoS0/1)              | Run control command (`start/stop/hold/reset`) with `cmd_id`. |
| MCU → HMI | `run/ack` (QoS0/1)              | Ack for `run/cmd` with `cmd_id/ok/err` + run state/permissions. |

Health aggregation rules (implemented in `HealthManager`):

- Any **required** component in `MISSING/ERROR/STALE` ⇒ `system_state=FAULT`, `run_allowed=false`, `outputs_allowed=false`.
- Optional component failure ⇒ `system_state=DEGRADED`, `run_allowed=true`, `outputs_allowed=true`.
- `UNCONFIGURED` does not affect the aggregate state.
- The DIN component is marked **required**; estop/lid/door faults drive `system_state=FAULT`.

Run control applies operator commands on top of health + DIN interlocks, with priority:
DIN estop > required health faults > operator command state.

The HMI should honor `run_allowed`/`outputs_allowed` when implementing run/estop logic and IO control.

### IO topics and payloads

- **DIN mapping (bit numbers, already inversion-adjusted):**
  - Bit 0 – Estop OK
  - Bit 1 – Lid locked
  - Bit 2 – Door closed
  - Bits 3–7 – spare digital inputs
  - The firmware enables `DIN_Inverse_Enable`, so a closed/active input reads as `1` in the mask.

- `io/din/state` (MCU → HMI, QoS0): published every `IO_STATE_PERIOD_MS` (200 ms)

  ```json
  { "v": 1, "ts_ms": 123456, "src": "esp32a", "mask": 7 }
  ```

- `io/din/event` (MCU → HMI, QoS0): on any change/edge

  ```json
  {
    "v": 1, "ts_ms": 123456, "src": "esp32a",
    "mask": 5, "prev_mask": 7,
    "rising": 0, "falling": 2
  }
  ```

- `io/cmd/event` (HMI → MCU, QoS0/1): set outputs by full mask or per-channel

  ```json
  { "cmd_id": 42, "mask": 129 }              // set all 8 relays at once
  { "cmd_id": 43, "channel": 3, "state": false }  // turn CH3 off (channels are 1–8)
  ```

  Commands are rejected with `outputs_inhibited` if `outputs_allowed=false` (health fault or DIN interlock).

- `io/dout/state` (MCU → HMI, QoS0): relay snapshot + permissions at 5 Hz

  ```json
  { "v": 1, "ts_ms": 123456, "src": "esp32a", "mask": 17, "outputs_allowed": true }
  ```

- `io/dout/ack` (MCU → HMI, QoS0/1): acknowledgement for `io/cmd/event`

  ```json
  {
    "v": 1, "ts_ms": 123460, "src": "esp32a",
    "cmd_id": 42, "ok": true,
    "mask": 129, "outputs_allowed": true
  }
  ```

### Run control topics and payloads

- `run/cmd` (HMI → MCU, QoS0/1): run control command

  ```json
  { "cmd_id": 10, "cmd": "start" }
  { "cmd_id": 11, "cmd": "hold" }
  { "cmd_id": 12, "cmd": "stop" }
  { "cmd_id": 13, "cmd": "reset" }
  ```

  Commands are rejected with `inhibited` when an estop or required health fault is latched. `reset`
  is rejected with `reset_inhibited` unless interlocks and required health are OK.

- `run/ack` (MCU → HMI, QoS0/1): acknowledgement for `run/cmd`

  ```json
  {
    "v": 1, "ts_ms": 123460, "src": "esp32a",
    "cmd_id": 10, "ok": true,
    "state": "RUNNING", "reason": "operator_start",
    "run_allowed": true, "outputs_allowed": true
  }
  ```

- `status/health` additions (MCU → HMI, QoS0): run state and reason

  ```json
  {
    "v": 1, "ts_ms": 123456, "src": "esp32a",
    "system_state": "OK",
    "run_state": "HOLDING",
    "run_reason": "operator_hold",
    "inhibit": { "run_allowed": false, "outputs_allowed": true },
    "summary": { "warn_count": 0, "crit_count": 0 }
  }
  ```

---

## Legacy HMI command set (v0 draft)

The original dashboard prototype used the `mill/*` topic map below. The PlatformIO firmware now implements run/estop control under the `<MACHINE_ID>/<NODE_ID>/` root (`run/cmd`, `run/ack`, and `status/health`), so treat the legacy map as deprecated and migrate any remaining dashboards accordingly.

---

### 1. Roles and Transport

#### 1.1 Roles

- **Broker**: Mosquitto running on the Raspberry Pi (port `1883`).
- **MCU**: ESP32-S3-ETH-8DI-8RO board.
- **HMI**: Node-RED + FlowFuse Dashboard running on the Pi.

#### 1.2 Connections

- **Node-RED → Broker**: always connects to `localhost:1883`.
- **MCU → Broker**: connects to the Pi’s Ethernet IP on the private link
  (e.g. `192.168.50.2:1883`).

#### 1.3 MQTT Settings

- Protocol: **MQTT v3.1.1**
- QoS: `0` (at-most-once) for all messages (v0).
- Retained: `false` for all messages (v0).

---

### 2. Topic Overview

All topics are prefixed with `mill/`.

| Direction      | Topic                 | Description                              |
|----------------|-----------------------|------------------------------------------|
| HMI → MCU      | `mill/cmd/control`    | High-level control commands              |
| HMI → MCU      | `mill/cmd/config`     | Run / cool times, cycle targets, etc.    |
| MCU → HMI      | `mill/status/state`   | Primary machine state snapshot           |
| MCU → HMI      | `mill/status/diag`    | Optional diagnostic / detailed status    |

Listeners can wildcard-subscribe to:

- `mill/#` for everything, or
- `mill/status/#` for read-only observers.

---

### 3. Control Commands (`mill/cmd/control`)

#### 3.1 Payload Format

**Topic:** `mill/cmd/control`  
**Direction:** HMI → MCU

```json
{
  "cmd": "START",
  "source": "HMI",
  "ts": 1764147000
}
```

#### Fields

- `cmd` (string, required) – one of:
  - `"START"` – start (or resume) the active program / cycle sequence.
  - `"STOP"` – immediate stop; transition to `IDLE` (may be treated as emergency stop at v0).
  - `"HOLD"` – pause current cycle, keep system in a safe but recoverable state.
  - `"RESUME"` – resume from `HOLD`.
  - `"RESET_FAULT"` – clear a latched fault *if* interlocks/conditions allow it.

- `source` (string, optional) – identifier for the origin:
  - `"HMI"` – Pi touchscreen dashboard.
  - `"REMOTE"` – future external client.
  - Can be ignored by MCU logic but useful for logging.

- `ts` (number, optional) – Unix timestamp (seconds since epoch) from the sender.
  - MCU may ignore this field and use its own clock.

#### 3.2 MCU Behaviour (high-level, v0)

- MCU subscribes to `mill/cmd/control`.
- On valid `cmd`, updates internal state machine and physical outputs.
- On invalid / unknown `cmd`, *ignores* the command and may optionally publish a warning in `mill/status/diag`.

---

### 4. Configuration Commands (`mill/cmd/config`)

(Used for setting program parameters from the HMI.)

**Topic:** `mill/cmd/config`  
**Direction:** HMI → MCU

```json
{
  "run_time_s": 300,
  "cool_time_s": 120,
  "cycle_target": 5,
  "ln2_sv_c": -90.0
}
```

#### Fields

- `run_time_s` (number, optional) – shaker run time per cycle in seconds.
- `cool_time_s` (number, optional) – cooling time per cycle in seconds.
- `cycle_target` (number, optional) – total number of run/cool cycles.
- `ln2_sv_c` (number, optional) – nominal setpoint for the LN₂ region (°C),
  used by MCU logic and/or written to the corresponding PID.

Any field may be omitted; the MCU should only update parameters that are present.

---

### 5. Primary Status (`mill/status/state`)

This is the **authoritative snapshot** of the mill’s state, published by the MCU at a regular interval (e.g. 5–10 Hz).

**Topic:** `mill/status/state`  
**Direction:** MCU → HMI

#### 5.1 Example

```json
{
  "ts": 1764146710,
  "state": "RUN",
  "substate": "RUN_ACTIVE",

  "cycle_current": 2,
  "cycle_target": 5,

  "run_time_s": 300,
  "cool_time_s": 120,

  "time_in_state_s": 42,
  "time_remaining_s": 258,

  "interlocks": {
    "door_closed": true,
    "estop_ok": true,
    "lid_locked": true
  },

  "pid": {
    "pv_c": -85.2,
    "sv_c": -90.0,
    "alarm": false
  },

  "heartbeat": 1
}
```

#### 5.2 Fields

##### 5.2.1 Top-level

- `ts` (number) – MCU timestamp (Unix seconds).

- `state` (string) – coarse machine state, one of:
  - `"IDLE"` – stopped, ready but not running a program.
  - `"RUN"` – actively executing the run/cool cycle sequence.
  - `"HOLD"` – paused mid-program; can normally resume.
  - `"FAULT"` – latched fault; requires RESET_FAULT and/or operator intervention.

- `substate` (string) – finer breakdown of the current high-level state.  
  Initial v0 suggestions (can be extended later):

  - For `IDLE`:
    - `"IDLE_READY"`
  - For `RUN`:
    - `"RUN_ACTIVE"` – shaker running.
    - `"RUN_COOLING"` – run phase complete, cooling countdown in progress.
  - For `HOLD`:
    - `"HOLD_USER"` – paused via command (HOLD).
    - `"HOLD_INTERLOCK"` – paused automatically due to a transient interlock.
  - For `FAULT`:
    - `"FAULT_INTERLOCK"` – door / estop / lid problem.
    - `"FAULT_DEVICE"` – PID / power / comms fault.
    - `"FAULT_INTERNAL"` – internal MCU error.

  HMI logic should treat unknown substates as generic members of `state`.

- `cycle_current` (number) – 0-based or 1-based indicator of which cycle is in progress (convention to be fixed in MCU).
- `cycle_target` (number) – total number of cycles configured (from config or local defaults).

- `run_time_s` (number) – configured run time per cycle (seconds).
- `cool_time_s` (number) – configured cool time per cycle (seconds).

- `time_in_state_s` (number) – seconds since the current `state`/`substate` began.
- `time_remaining_s` (number) – remaining seconds in the relevant phase (run or cool), or `0` if not applicable.

- `heartbeat` (number) – monotonically increasing counter for connectivity diagnostics.

##### 5.2.2 Interlocks

```json
"interlocks": {
  "door_closed": true,
  "estop_ok": true,
  "lid_locked": true
}
```

- `door_closed` (bool) – true if primary door interlock loop is satisfied.
- `estop_ok` (bool) – true if estop chain is not tripped (DCOK / monitoring signal present).
- `lid_locked` (bool) – true if jar/door lock solenoid & confirmation switch indicate locked.

Additional interlocks (lid temperature, shaker overtemp, etc.) can be added later as extra fields.

##### 5.2.3 LN₂ / temperature PID summary

```json
"pid": {
  "pv_c": -85.2,
  "sv_c": -90.0,
  "alarm": false
}
```

- `pv_c` (number) – measured process value from the LN₂ PID (°C).
- `sv_c` (number) – setpoint (°C) that the PID is currently targeting.
- `alarm` (bool) – true if the PID signals any active alarm condition
  (sensor fault, out-of-range, etc.).

In future, this may expand to an object per PID device, e.g. `pid_ln2`, `pid_base`, `pid_bearing`, etc.

---

### 6. Diagnostics (`mill/status/diag`) – optional, v0

This topic is optional and for verbose info that doesn’t need to drive the HMI directly (counters, error strings, device online state, etc.).

**Topic:** `mill/status/diag`  
**Direction:** MCU → HMI / logger

Example (non-final):

```json
{
  "ts": 1764146710,
  "fault_code": 0,
  "fault_msg": "",
  "devices": {
    "pid_ln2": { "online": true },
    "pid_base": { "online": true },
    "pid_bearing": { "online": false }
  },
  "comm": {
    "rs485_errors": 0,
    "mqtt_reconnects": 1
  }
}
```

HMI may display some of this in an “Advanced / Diagnostics” view; most clients can ignore it.

---

### 7. Versioning and Extensibility

- This document defines **protocol v0**.
- Future changes should:
  - **Add** new fields rather than change the meaning of existing ones.
  - Use **sensible defaults** when fields are missing.
  - Consider adding a `protocol_version` field to `mill/status/state` if we make incompatible changes later.

For now, the MCU and HMI are assumed to be in lockstep; if the schema changes, both sides will be updated together.
