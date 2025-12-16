# Board configuration notes

This PlatformIO project uses a **project-local** custom board definition:

- `boards/esp32-s3-devkitc-1-n16r8v.json`

PlatformIO will prefer the project-local `boards/` directory when resolving `board = ...`
and will apply that JSON's `build` / `upload` settings automatically.

Goal for initial bring-up:
- Keep `platformio.ini` minimal (no redundant `board_build.*` overrides) so that the board JSON
  is the source of truth for flash/PSRAM sizing and modes.
- Only add project-level overrides (like a custom partitions CSV) once baseline bring-up is proven.
