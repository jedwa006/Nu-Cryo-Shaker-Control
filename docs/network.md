## Ethernet configuration

The firmware targets the Waveshare ESP32-S3-ETH-8DI-8RO with a W5500. Ethernet settings live in `firmware_esp32s3_pio/nu_cryo_control/include/app/app_config.h`.

### Static IP versus DHCP

The `ETH_STATIC_*` constants define the controller’s static address:

```cpp
static constexpr Ip4 ETH_STATIC_IP   = {192,168,50,10};
static constexpr Ip4 ETH_STATIC_GW   = {192,168,50,2};
static constexpr Ip4 ETH_STATIC_MASK = {255,255,255,0};
```

* **DHCP fallback:** If `ETH_STATIC_IP` is set to `0.0.0.0`, the firmware skips static configuration and requests an address via DHCP.
* **Applying static settings:** When any non-zero static IP is present, `ETH.config(...)` is invoked with the provided IP, gateway, and subnet mask. If the call succeeds, the firmware records that static addressing is active; if it fails, the firmware logs the error and falls back to DHCP.

### Boot telemetry

On successful MQTT connection the boot message includes:

* `eth`: `true` when the link is up and an IP address is assigned.
* `eth_static`: `true` when `ETH.config` succeeded and static addressing is in use.
* `ip`: the assigned IP (present only when `eth` is `true`).

These fields are published under the boot topic `<MACHINE_ID>/<NODE_ID>/status/boot`.

### MQTT broker defaults

By default the firmware connects to `192.168.50.2:1883`. Adjust `MQTT_BROKER_HOST` and `MQTT_BROKER_PORT` in `app_config.h` if your broker runs elsewhere.

### Machine and node IDs

Root topics are prefixed by `<MACHINE_ID>/<NODE_ID>/`. Set `MACHINE_ID` to the installation name (e.g., the specific cryo mill) and `NODE_ID` to the ESP32S3 module identifier when deploying multiple nodes on the same network.
