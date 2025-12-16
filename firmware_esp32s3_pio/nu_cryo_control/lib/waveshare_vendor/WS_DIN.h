#pragma once

#include "WS_GPIO.h"
#include "WS_Relay.h"
/*************************************************************  I/O  *************************************************************/
#define DIN_PIN_CH1      4      // DIN CH1 GPIO
#define DIN_PIN_CH2      5      // DIN CH2  GPIO
#define DIN_PIN_CH3      6      // DIN CH3  GPIO
#define DIN_PIN_CH4      7      // DIN CH4  GPIO
#define DIN_PIN_CH5      8      // DIN CH5  GPIO
#define DIN_PIN_CH6      9      // DIN CH6  GPIO
#define DIN_PIN_CH7      10     // DIN CH7  GPIO
#define DIN_PIN_CH8      11     // DIN CH8  GPIO

#define Relay_Immediate_Default   1       // Enable the input control relay
#define DIN_Inverse_Enable        1       // Input is reversed from control

void DIN_Init(void);
