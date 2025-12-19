#pragma once
#include <stdint.h>

struct Ip4 { uint8_t a, b, c, d; };

struct BoardPins {
  int w5500_cs;
  int w5500_int;
  int w5500_rst;
  int w5500_sck;
  int w5500_miso;
  int w5500_mosi;
};

struct NetDefaults {
  const char* mqtt_broker_host;
  uint16_t mqtt_broker_port;
  const char* mqtt_username;
  const char* mqtt_password;
  Ip4 static_ip;
  Ip4 static_gw;
  Ip4 static_mask;
};

struct NetTopics {
  const char* lwt;
  const char* boot;
  const char* health;
};

struct ModbusConfig {
  uint32_t baud;
  int tx_pin;
  int rx_pin;
  int de_re_pin;
  uint8_t pid_heat1_id;
  uint8_t pid_heat2_id;
  uint8_t pid_cool1_id;
};

#ifndef NUCRYO_WAVESHARE_RS485_DE_RE_PIN
#define NUCRYO_WAVESHARE_RS485_DE_RE_PIN -1
#endif

inline constexpr BoardPins BOARD_WAVESHARE_PINS{
  .w5500_cs = 16,   // ETH_PHY_CS
  .w5500_int = 12,  // ETH_PHY_IRQ
  .w5500_rst = 39,  // ETH_PHY_RST
  .w5500_sck = 15,
  .w5500_miso = 14,
  .w5500_mosi = 13,
};

inline constexpr NetDefaults BOARD_WAVESHARE_NET{
  .mqtt_broker_host = "192.168.50.2",
  .mqtt_broker_port = 1883,
  .mqtt_username = "",
  .mqtt_password = "",
  .static_ip = {192, 168, 50, 10},
  .static_gw = {192, 168, 50, 2},
  .static_mask = {255, 255, 255, 0},
};

inline constexpr NetTopics BOARD_WAVESHARE_TOPICS{
  .lwt = "status/lwt",
  .boot = "status/boot",
  .health = "status/health",
};

inline constexpr ModbusConfig BOARD_WAVESHARE_MODBUS{
  .baud = 9600,
  .tx_pin = 17,
  .rx_pin = 18,
  // NOTE: Avoid USB D- (GPIO19) on ESP32-S3; override with a build flag if needed.
  .de_re_pin = NUCRYO_WAVESHARE_RS485_DE_RE_PIN,
  .pid_heat1_id = 1,
  .pid_heat2_id = 2,
  .pid_cool1_id = 3,
};
