#include "WS_Serial.h"
void Serial_Init()
{
  if(RS485_CAN_Enable)
    RS485_Init();
  else
    CAN_Init();
}
