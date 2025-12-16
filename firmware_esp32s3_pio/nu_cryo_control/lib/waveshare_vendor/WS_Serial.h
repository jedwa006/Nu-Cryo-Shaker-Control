#pragma once

#include "WS_Information.h"
#include "WS_RS485.h"
#include "WS_CAN.h"

void Serial_Init();         // Example Initialize the system serial port and RS485
void Serial_Loop();         // Read RS485 data, parse and control relays
