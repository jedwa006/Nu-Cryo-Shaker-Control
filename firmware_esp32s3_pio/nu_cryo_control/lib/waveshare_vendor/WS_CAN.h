#pragma once

#include "driver/twai.h"
#include "WS_GPIO.h"

// Interval:
#define TRANSMIT_RATE_MS      1000
// Interval:
#define POLLING_RATE_MS       1000

#define Communication_failure_Enable    0         // If the CAN bus is faulty for a long time, determine whether to forcibly exit

#if Communication_failure_Enable
  #define BUS_ERROR_INTERVAL_MS         5000      // Send a message every 2 seconds (2000 ms)
#endif


void CAN_Init(void);
void CAN_Loop(void);
void CANTask(void *parameter);

void send_message(uint32_t CAN_ID, uint8_t* Data, uint8_t Data_length);