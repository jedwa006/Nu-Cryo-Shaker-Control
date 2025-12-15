# Nu-Cryo PlatformIO firmware scaffold (repo layout)

Drop this tree into the root of your repository.

PlatformIO project path:
`firmware_esp32s3_pio/nu_cryo_control/`

Next steps:
1. Open `firmware_esp32s3_pio/nu_cryo_control` in VS Code + PlatformIO.
2. Edit `include/app/app_config.h`:
   - W5500 pins, broker IP, Modbus pins
   - Modbus register map placeholders (PV/SV/out%)
3. Copy Waveshare vendor sources to:
   - `lib/waveshare_vendor/`
4. Implement wrappers in:
   - `lib/waveshare_hal/`
