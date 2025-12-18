#pragma once
#include <stdint.h>

// -------- Identity / topics --------
// Root topic: <MACHINE_ID>/<NODE_ID>/...
static constexpr const char* MACHINE_ID = "cryo_mill_01";
static constexpr const char* NODE_ID    = "esp32a";

// -------- MQTT broker (on your Pi) --------
// If using Ethernet with static addressing, set broker_ip explicitly.
// If DHCP, your broker hostname may work if DNS is configured.
static constexpr const char* MQTT_BROKER_HOST = "192.168.50.2";
static constexpr uint16_t    MQTT_BROKER_PORT = 1883;

static constexpr const char* MQTT_USERNAME = "";   // optional
static constexpr const char* MQTT_PASSWORD = "";   // optional

// Heartbeat + publish rates
static constexpr uint32_t HEARTBEAT_PERIOD_MS      = 1000;  // sys/heartbeat
static constexpr uint32_t SYS_HEALTH_PERIOD_MS     = 1000;  // sys/health
static constexpr uint32_t COMPONENT_HEALTH_PERIOD_MS=1000;  // health/<component>/state
static constexpr uint32_t PID_STATE_PERIOD_MS      = 200;   // 5 Hz panel-emulation
static constexpr uint32_t PID_PARAMS_PERIOD_MS     = 5000;  // slow
static constexpr uint32_t IO_STATE_PERIOD_MS       = 200;

// -------- Ethernet (W5500) --------
// Waveshare ESP32-S3-ETH-8DI-8RO wiring (from WS_ETH.h)
static constexpr int W5500_CS_PIN   = 16;  // ETH_PHY_CS
static constexpr int W5500_INT_PIN  = 12;  // ETH_PHY_IRQ
static constexpr int W5500_RST_PIN  = 39;  // ETH_PHY_RST

// Dedicated SPI bus pins used on the Waveshare board for W5500
static constexpr int W5500_SCK_PIN  = 15;
static constexpr int W5500_MISO_PIN = 14;
static constexpr int W5500_MOSI_PIN = 13;


// Optional: static IP for the ESP side. If all zeros, DHCP is used.
struct Ip4 { uint8_t a,b,c,d; };
static constexpr Ip4 ETH_STATIC_IP   = {192,168,50,10};
static constexpr Ip4 ETH_STATIC_GW   = {192,168,50,2};
static constexpr Ip4 ETH_STATIC_MASK = {255,255,255,0};   // typical /24

// -------- Modbus RTU (PID controllers) --------
static constexpr uint32_t MODBUS_BAUD = 115200;
static constexpr int MODBUS_TX_PIN = -1;  // TODO: set
static constexpr int MODBUS_RX_PIN = -1;  // TODO: set
static constexpr int MODBUS_DE_RE_PIN = -1; // TODO: set (driver enable)

// Example slave IDs (adjust)
static constexpr uint8_t PID_HEAT1_ID = 1;
static constexpr uint8_t PID_HEAT2_ID = 2;
static constexpr uint8_t PID_COOL1_ID = 3;

// Register map placeholders (adjust to your controller)
// You can keep these grouped per controller type later if you support multiple PID brands/models.
static constexpr uint16_t REG_PV = 0x0000;      // TODO
static constexpr uint16_t REG_SV = 0x0001;      // TODO
static constexpr uint16_t REG_OUT_PCT = 0x0002; // TODO
