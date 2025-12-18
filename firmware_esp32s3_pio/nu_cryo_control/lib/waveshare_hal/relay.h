#pragma once

#include <stdint.h>

// Wrapper for the Waveshare relay/GPIO expander. Keeps vendor headers private
// to this HAL and exposes a simple bitmask interface.
class RelayHal {
public:
  bool begin();

  bool write_mask(uint8_t mask);
  bool read_mask(uint8_t& mask);

private:
  bool initialized_ {false};
};
