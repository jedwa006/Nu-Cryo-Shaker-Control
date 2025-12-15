#pragma once
#include <ETH.h>
#include <SPI.h>

#include "WS_PCF85063.h"
#include "WS_GPIO.h"
#include "WS_RTC.h"

// Set this to 1 to enable dual Ethernet support
#define USE_TWO_ETH_PORTS 0

#ifndef ETH_PHY_TYPE
  #define ETH_PHY_TYPE ETH_PHY_W5500
  #define ETH_PHY_ADDR 1
  #define ETH_PHY_CS   16
  #define ETH_PHY_IRQ  12
  #define ETH_PHY_RST  39
#endif

// SPI pins
#define ETH_SPI_SCK  15
#define ETH_SPI_MISO 14
#define ETH_SPI_MOSI 13

#if USE_TWO_ETH_PORTS
  // Second port on shared SPI bus
  #ifndef ETH1_PHY_TYPE
    #define ETH1_PHY_TYPE ETH_PHY_W5500
    #define ETH1_PHY_ADDR 1
    #define ETH1_PHY_CS   32
    #define ETH1_PHY_IRQ  33
    #define ETH1_PHY_RST  18
  #endif
  ETHClass ETH1(1);
#endif

#define timezone 8        // china 

void ETH_Init(void);
void ETH_Loop(void);
void EthernetTask(void *parameter);

void Acquisition_time(void);