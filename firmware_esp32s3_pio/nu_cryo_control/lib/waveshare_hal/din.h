#pragma once

#include <stdint.h>

// Thin wrapper around the vendor DIN helpers. Provides simple masked reads
// with optional edge detection while keeping the vendor headers private to
// the HAL boundary.
class DinHal {
public:
  bool begin();

  // Read all channels into a bitmask (LSB = CH1). Optional rising/falling
  // masks are zeroed on the first call to avoid spurious events.
  uint8_t read_all(uint8_t* rising_edges = nullptr, uint8_t* falling_edges = nullptr);

  uint8_t last_mask() const { return last_mask_; }

private:
  bool initialized_ {false};
  bool first_read_ {true};
  uint8_t last_mask_ {0};
};
