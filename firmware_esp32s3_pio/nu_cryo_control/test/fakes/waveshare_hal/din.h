#pragma once

#include <stdint.h>

class DinHal {
public:
  bool begin() { return true; }

  uint8_t read_all(uint8_t* rising_edges = nullptr, uint8_t* falling_edges = nullptr) {
    if (rising_edges != nullptr) {
      *rising_edges = 0;
    }
    if (falling_edges != nullptr) {
      *falling_edges = 0;
    }
    return 0;
  }

  uint8_t last_mask() const { return 0; }
};
