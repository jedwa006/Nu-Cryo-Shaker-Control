#pragma once
#include <stdint.h>
#include "config/board_waveshare.h"

// -------- Identity --------
// Root topic: <MACHINE_ID>/<NODE_ID>/...
static constexpr const char* MACHINE_ID = "cryo_mill_01";
static constexpr const char* NODE_ID    = "esp32a";

// -------- Board configuration --------
static constexpr BoardPins BOARD_PINS       = BOARD_WAVESHARE_PINS;
static constexpr NetDefaults NET_DEFAULTS   = BOARD_WAVESHARE_NET;
static constexpr NetTopics NET_TOPICS       = BOARD_WAVESHARE_TOPICS;
static constexpr ModbusConfig MODBUS_CONFIG = BOARD_WAVESHARE_MODBUS;

// Heartbeat + publish rates
static constexpr uint32_t HEARTBEAT_PERIOD_MS      = 1000;  // sys/heartbeat
static constexpr uint32_t SYS_HEALTH_PERIOD_MS     = 1000;  // sys/health
static constexpr uint32_t COMPONENT_HEALTH_PERIOD_MS=1000;  // health/<component>/state
static constexpr uint32_t PID_STATE_PERIOD_MS      = 200;   // 5 Hz panel-emulation
static constexpr uint32_t PID_PARAMS_PERIOD_MS     = 5000;  // slow
static constexpr uint32_t IO_STATE_PERIOD_MS       = 200;

// Register map for LC108 / COM-800-C1 compatible PID controllers.
// Register numbers are 1-based (per the controller docs).
namespace LC108 {
enum Reg : uint16_t {
  PV     = 1,   // Process value (measured temperature), x10
  MV1    = 2,   // Output 1 %, x10
  MV2    = 3,   // Output 2 %, x10
  MVFB   = 4,   // Feedback %, x10
  STATUS = 5,   // Status / output bitfield
  SV     = 6,   // Main setpoint, x10
  SV1    = 8,
  SV2    = 9,
  SV3    = 10,
  SV4    = 11,
  AT     = 13,
  MODE   = 14,
  AL1    = 15,
  AL2    = 16,
  SC     = 24,
  P1     = 25,
  I1     = 26,
  D1     = 27,
  CYT1   = 30,
  HYS1   = 31,
  RST1   = 32,
  OPL1   = 33,
  OPH1   = 34,
  BUF1   = 35,
  INP1   = 66,
  DP     = 67,
  UNIT   = 68,
  LSPL   = 69,
  USPL   = 70,
  PVOS   = 71,
  PVFT   = 72,
  ANL1   = 73,
  ANH1   = 74,
  ALD1   = 77,
  AH1    = 78,
  ALT1   = 79,
  ALD2   = 80,
  AH2    = 81,
  ALT2   = 82,
  PMD    = 92,
  TSP    = 93,
  PEND   = 94,
  IDNO   = 95,
  BAUD   = 96,
  UCR    = 97,
  EXC1   = 98,
  A1L1   = 99,
  EXC2   = 100,
  A1L2   = 101
};

inline float decode_temp(int16_t value) { return static_cast<float>(value) / 10.0f; }
inline int16_t encode_temp(float temp) { return static_cast<int16_t>(temp * 10.0f + 0.5f); }
inline float decode_percent(int16_t value) { return static_cast<float>(value) / 10.0f; }
inline int16_t encode_percent(float percent) { return static_cast<int16_t>(percent * 10.0f + 0.5f); }
} // namespace LC108

// Primary runtime registers used by the PID component (1-based).
static constexpr uint16_t REG_PV = LC108::PV;
static constexpr uint16_t REG_SV = LC108::SV;
static constexpr uint16_t REG_OUT_PCT = LC108::MV1;
static constexpr uint16_t REG_STATUS = LC108::STATUS;
static constexpr uint16_t REG_ALARM1 = LC108::AL1;
static constexpr uint16_t REG_ALARM2 = LC108::AL2;
static constexpr uint16_t REG_P1 = LC108::P1;
static constexpr uint16_t REG_I1 = LC108::I1;
static constexpr uint16_t REG_D1 = LC108::D1;
static constexpr uint16_t REG_OPL1 = LC108::OPL1;
static constexpr uint16_t REG_OPH1 = LC108::OPH1;
static constexpr uint16_t REG_LSPL = LC108::LSPL;
static constexpr uint16_t REG_USPL = LC108::USPL;
