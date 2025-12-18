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

// Register map placeholders (adjust to your controller)
// You can keep these grouped per controller type later if you support multiple PID brands/models.
static constexpr uint16_t REG_PV = 0x0000;      // TODO
static constexpr uint16_t REG_SV = 0x0001;      // TODO
static constexpr uint16_t REG_OUT_PCT = 0x0002; // TODO
