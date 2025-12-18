# Nu-Cryo MQTT Protocol (v1)

Root topic: `<MACHINE_ID>/<NODE_ID>/...`  
Example: `cryo_mill_01/esp32a/...`

## System / Health

- `.../sys/birth` (retained, QoS1): published on boot (online)
- `.../sys/heartbeat` (QoS0): 1 Hz
- `.../sys/capabilities` (retained, QoS1): what exists + what is expected/required
- `.../sys/health` (QoS0 or QoS1): aggregate health state (OK/DEGRADED/FAULT/SAFE_SHUTDOWN)

Per-component:
- `.../health/<component>/state` (QoS0): component health snapshot

### Health Status values
`UNCONFIGURED | MISSING | OK | DEGRADED | STALE | ERROR`

### Aggregate rules
- Any **required** component in `MISSING/ERROR/STALE` => `system_state = FAULT` and `run_allowed=false`
- Only **optional** components failing => `system_state = DEGRADED` and `run_allowed=true`
- `UNCONFIGURED` never affects aggregate state.

## Commands / ACK

Commands are sent to `.../<subsystem>/<name>/cmd/<verb>` with:
- `cmd_id` (uint32): caller-generated correlation id
- `ttl_ms` (optional): command expires if too old
- payload fields (e.g. `value`)

Device responds on `.../<subsystem>/<name>/ack` with:
- `cmd_id`, `ok`, `err` and any applied values.

## PIDs

- `.../pid/<name>/state` (QoS0): PV/SV/out%, alarms/bits at ~5Hz
- `.../pid/<name>/params` (QoS0/1): slow parameter block
- `.../pid/<name>/cmd/sv` (QoS1)
- `.../pid/<name>/cmd/mode` (QoS1)
- `.../pid/<name>/ack` (QoS1)

## IO / Safety

- `.../safety/state` and `.../safety/event`
- `.../io/din/state` (mask @ `IO_STATE_PERIOD_MS`), `.../io/din/event` (rising/falling/prev_mask)
- `.../io/cmd/event` (set relay mask or `channel` + `state`)
- `.../io/dout/state` (relay mask + `outputs_allowed`) and `.../io/dout/ack` (cmd_id/ok/err)
- DIN bits 0–2 map to estop OK / lid locked / door closed (inversion enabled). These are marked required and gate `outputs_allowed`.
