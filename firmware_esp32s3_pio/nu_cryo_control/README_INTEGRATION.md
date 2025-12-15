# Integration Notes

This is a clean, modular PlatformIO skeleton that you can drop into your existing repo
alongside your current `minimal_mqtt_bridge`.

Recommended approach:
1. Create a new folder in your repo, e.g.:
   `firmware ESP32S3/nucryo_controller_pio/`
   and copy this project into it.

2. Copy any Waveshare vendor "API-like" files into:
   `lib/waveshare_vendor/` (or keep them under `board/` and include them from `waveshare_hal.*`).

3. Implement a thin `waveshare_hal` wrapper that:
   - sets correct W5500 pins
   - sets correct RS-485 TX/RX/DE/RE pins
   - exposes digital input read functions for DIN / e-stop
   Keep the rest of the firmware unaware of vendor specifics.

4. Adjust the PID Modbus register map in `app/app_config.h`:
   `REG_PV`, `REG_SV`, `REG_OUT_PCT` and scaling.

5. Enable optional sensors:
   In `platformio.ini` set:
   -D NUCRYO_ENABLE_ADXL345=1
   -D NUCRYO_ENABLE_MLX90640=1 (not included; stub can be added)

Key behavior:
- Missing optional sensors => system publishes DEGRADED but run_allowed remains true.
- Missing required components (PIDs, safety) => system publishes FAULT and run_allowed=false.

