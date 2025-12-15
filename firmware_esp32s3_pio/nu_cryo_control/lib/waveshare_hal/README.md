# waveshare_hal

Thin board abstraction layer for the Waveshare ESP32-S3 ETH-8DI-8RO board.

Provide a small, stable API for:
- W5500 init (SPI + CS/INT/RST pins)
- RS-485 UART init (TX/RX + DE/RE)
- DIN/DOUT access (GPIO expanders / opto modules)
- Any board LEDs / status pins you want standardized

The rest of the firmware should depend only on this layer (not vendor headers).
