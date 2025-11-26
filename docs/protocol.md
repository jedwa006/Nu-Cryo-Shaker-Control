# Cryo Mill Control – MQTT Protocol (v0)

This document defines the MQTT topics, message formats, and basic state
machine used by the cryogenic ball mill control system.

The protocol is designed to keep:

- **MCU (ESP32-S3)** as the *single source of truth* for process state,
  interlocks, and device status.
- **Pi / Node-RED / Dashboard** as a *thin client* that:
  - sends high-level commands, and
  - displays whatever the MCU reports.

All other tools (remote monitor, logger, etc.) should speak only this protocol.

---

## 1. Roles and Transport

### 1.1 Roles

- **Broker**: Mosquitto running on the Raspberry Pi (port `1883`).
- **MCU**: ESP32-S3-ETH-8DI-8RO board.
- **HMI**: Node-RED + FlowFuse Dashboard running on the Pi.

### 1.2 Connections

- **Node-RED → Broker**: always connects to `localhost:1883`.
- **MCU → Broker**: connects to the Pi’s Ethernet IP on the private link
  (e.g. `192.168.50.2:1883`).

### 1.3 MQTT Settings

- Protocol: **MQTT v3.1.1**
- QoS: `0` (at-most-once) for all messages (v0).
- Retained: `false` for all messages (v0).

---

## 2. Topic Overview

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

## 3. Control Commands (`mill/cmd/control`)

### 3.1 Payload Format

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

### 3.2 MCU Behaviour (high-level, v0)

- MCU subscribes to `mill/cmd/control`.
- On valid `cmd`, updates internal state machine and physical outputs.
- On invalid / unknown `cmd`, *ignores* the command and may optionally publish a warning in `mill/status/diag`.

---

## 4. Configuration Commands (`mill/cmd/config`)

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

## 5. Primary Status (`mill/status/state`)

This is the **authoritative snapshot** of the mill’s state, published by the MCU at a regular interval (e.g. 5–10 Hz).

**Topic:** `mill/status/state`  
**Direction:** MCU → HMI

### 5.1 Example

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

### 5.2 Fields

#### 5.2.1 Top-level

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

#### 5.2.2 Interlocks

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

#### 5.2.3 LN₂ / temperature PID summary

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

## 6. Diagnostics (`mill/status/diag`) – optional, v0

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

## 7. Versioning and Extensibility

- This document defines **protocol v0**.
- Future changes should:
  - **Add** new fields rather than change the meaning of existing ones.
  - Use **sensible defaults** when fields are missing.
  - Consider adding a `protocol_version` field to `mill/status/state` if we make incompatible changes later.

For now, the MCU and HMI are assumed to be in lockstep; if the schema changes, both sides will be updated together.
