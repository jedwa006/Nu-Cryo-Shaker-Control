#pragma once

#include <HardwareSerial.h>     // Reference the ESP32 built-in serial port library

/*************************************************************  I/O  *************************************************************/
#define TXD1              17    //The TXD of UART1 corresponds to GPIO   RS485/CAN
#define RXD1              18    //The RXD of UART1 corresponds to GPIO   RS485/CAN
#define GPIO_PIN_RGB      38    // RGB Control GPIO

/***********************************************************  Buzzer  ***********************************************************/
#define GPIO_PIN_Buzzer 46      // Buzzer Control GPIO
#define PWM_Channel     1       // PWM Channel   
#define Frequency       1000    // PWM frequencyconst
#define Resolution      8       // PWM resolution ratio
#define Dutyfactor      200     // PWM Dutyfactor
#define Dutyfactor_MAX  255


#define RGB_Indicate_Number  10         // Number of saved RGB indicator signals
#define RGB_Indicating_interval 500     // Time interval of each indication signal(unit: ms)
typedef struct { 
  uint8_t Red = 0;
  uint8_t Green = 0;
  uint8_t Blue = 0;
  uint16_t RGB_Time = 0;                // RGB lighting duration
  uint16_t RGB_Flicker = 0;             // RGB flicker interval
} RGB_Indicate;

#define Buzzer_Indicate_Number  10      // Number of saved RGB indicator signals
typedef struct { 
  uint16_t Buzzer_Time = 0;             // Buzzer duration
  uint16_t Buzzer_Flicker = 0;          // Buzzer interval duration
} Buzzer_Indicate;
/*************************************************************  I/O  *************************************************************/
void GPIO_Init();
void RGB_Light(uint8_t red_val, uint8_t green_val, uint8_t blue_val);
void RGB_Open_Time(uint8_t red_val, uint8_t green_val, uint8_t blue_val, uint16_t Time, uint16_t flicker_time);
void RGBTask(void *parameter);

void Set_Dutyfactor(uint16_t dutyfactor);
void Buzzer_Open(void);
void Buzzer_Closs(void);
void Buzzer_Open_Time(uint16_t Time, uint16_t flicker_time); 
void BuzzerTask(void *parameter);
