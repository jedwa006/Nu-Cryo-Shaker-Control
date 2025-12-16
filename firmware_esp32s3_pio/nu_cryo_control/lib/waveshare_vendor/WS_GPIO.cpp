#include "WS_GPIO.h"

/*************************************************************  I/O Init  *************************************************************/
void GPIO_Init() {
  pinMode(GPIO_PIN_RGB, OUTPUT);     // Initialize the control GPIO of RGB
  pinMode(GPIO_PIN_Buzzer, OUTPUT);  // Initialize the control GPIO of Buzzer

  ledcAttach(GPIO_PIN_Buzzer, Frequency, Resolution);   
  Set_Dutyfactor(0);                //0~100  

  xTaskCreatePinnedToCore(
    RGBTask,    
    "RelayFailTask",   
    4096,                
    NULL,                 
    2,                   
    NULL,                 
    0                   
  );
  xTaskCreatePinnedToCore(
    BuzzerTask,    
    "RelayFailTask",   
    4096,                
    NULL,                 
    2,                   
    NULL,                 
    0                   
  );
}

/*************************************************************  RGB  *************************************************************/
void RGB_Light(uint8_t red_val, uint8_t green_val, uint8_t blue_val) {
  neopixelWrite(GPIO_PIN_RGB, green_val, red_val, blue_val);  // RGB color adjustment
}
RGB_Indicate RGB_indicate[RGB_Indicate_Number];
static uint8_t RGB_indicate_Num = 0;
void RGB_Open_Time(uint8_t red_val, uint8_t green_val, uint8_t blue_val, uint16_t Time, uint16_t flicker_time) {

  if(RGB_indicate_Num + 1 >= RGB_Indicate_Number)
  {
    printf("Note : The RGB indicates that the cache is full and has been ignored\r\n");
  }
  else{
    RGB_indicate[RGB_indicate_Num].Red = red_val;
    RGB_indicate[RGB_indicate_Num].Green = green_val;
    RGB_indicate[RGB_indicate_Num].Blue = blue_val;
    RGB_indicate[RGB_indicate_Num].RGB_Time = Time;
    if(flicker_time<51)
      flicker_time = 0;                                         // If the blinking interval is less than 50ms, the blinking is ignored
    RGB_indicate[RGB_indicate_Num].RGB_Flicker = flicker_time;
    RGB_indicate_Num ++;
  }
}
void RGBTask(void *parameter) {
  bool RGB_Flag = 0;
  while(1){
    if(RGB_indicate[0].RGB_Time)
    {
      RGB_Flag = 1;
      RGB_Light(RGB_indicate[0].Red, RGB_indicate[0].Green, RGB_indicate[0].Blue); 
      if(RGB_indicate[0].RGB_Flicker){
        vTaskDelay(pdMS_TO_TICKS(RGB_indicate[0].RGB_Flicker));
        RGB_Light(0, 0, 0);  
        vTaskDelay(pdMS_TO_TICKS(RGB_indicate[0].RGB_Flicker));
      }
      if(RGB_indicate[0].RGB_Time > (RGB_indicate[0].RGB_Flicker * 2 +50))
        RGB_indicate[0].RGB_Time = RGB_indicate[0].RGB_Time -(RGB_indicate[0].RGB_Flicker * 2 +50);
      else
        RGB_indicate[0].RGB_Time = 0;
    }
    else if(RGB_Flag && !RGB_indicate[0].RGB_Time){
      RGB_Light(0, 0, 0);  
      RGB_Flag = 0;
      RGB_indicate[0].Red = 0;
      RGB_indicate[0].Green = 0;
      RGB_indicate[0].Blue = 0;
      RGB_indicate[0].RGB_Time = 0;
      RGB_indicate[0].RGB_Flicker = 0;
      if(RGB_indicate_Num > 0){
        for (int i = 1; i < RGB_Indicate_Number; i++) {
          RGB_indicate[i-1] = RGB_indicate[i];  
        }
        RGB_indicate[RGB_Indicate_Number -1].Red = 0;
        RGB_indicate[RGB_Indicate_Number -1].Green = 0;
        RGB_indicate[RGB_Indicate_Number -1].Blue = 0;
        RGB_indicate[RGB_Indicate_Number -1].RGB_Time = 0;
        RGB_indicate[RGB_Indicate_Number -1].RGB_Flicker = 0;
        RGB_indicate_Num --;
        vTaskDelay(pdMS_TO_TICKS(RGB_Indicating_interval));
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  vTaskDelete(NULL);
}


/*************************************************************  Buzzer  *************************************************************/
void Set_Dutyfactor(uint16_t dutyfactor)                     
{
  if(dutyfactor > Dutyfactor_MAX || dutyfactor < 0)
    printf("Set Backlight parameters in the range of 0 to %d \r\n",Dutyfactor_MAX);
  else{
    ledcWrite(GPIO_PIN_Buzzer, dutyfactor);
  }
}
void Buzzer_Open(void)  
{
  Set_Dutyfactor(Dutyfactor);
}
void Buzzer_Closs(void)  
{
  Set_Dutyfactor(0);
}
Buzzer_Indicate Buzzer_indicate[Buzzer_Indicate_Number];
static uint8_t Buzzer_indicate_Num = 0;
void Buzzer_Open_Time(uint16_t Time, uint16_t flicker_time) 
{
  if(Buzzer_indicate_Num + 1 >= Buzzer_Indicate_Number)
  {
    printf("Note : The buzzer indicates that the cache is full and has been ignored\r\n");
  }
  else{
    Buzzer_indicate[Buzzer_indicate_Num].Buzzer_Time = Time;
    if(flicker_time<51)
      flicker_time = 0;                                         // If the blinking interval is less than 50ms, the blinking is ignored
    Buzzer_indicate[Buzzer_indicate_Num].Buzzer_Flicker = flicker_time;
    Buzzer_indicate_Num ++;
  }
}
void BuzzerTask(void *parameter) {
  bool Buzzer_Flag = 0;
  while(1){
    if(Buzzer_indicate[0].Buzzer_Time)
    {
      Buzzer_Flag = 1;
      Buzzer_Open(); 
      if(Buzzer_indicate[0].Buzzer_Flicker){
        vTaskDelay(pdMS_TO_TICKS(Buzzer_indicate[0].Buzzer_Flicker));
        Buzzer_Closs();  
        vTaskDelay(pdMS_TO_TICKS(Buzzer_indicate[0].Buzzer_Flicker));
      }
      if(Buzzer_indicate[0].Buzzer_Time > (Buzzer_indicate[0].Buzzer_Flicker * 2 +50))
        Buzzer_indicate[0].Buzzer_Time = Buzzer_indicate[0].Buzzer_Time -(Buzzer_indicate[0].Buzzer_Flicker * 2 +50);
      else
        Buzzer_indicate[0].Buzzer_Time = 0;
    }
    else if(Buzzer_Flag && !Buzzer_indicate[0].Buzzer_Time){
      Buzzer_Closs();  
      Buzzer_Flag = 0;
      Buzzer_indicate[0].Buzzer_Time = 0;
      Buzzer_indicate[0].Buzzer_Flicker = 0;
      if(Buzzer_indicate_Num > 0){
        for (int i = 1; i < Buzzer_indicate_Num; i++) {
          Buzzer_indicate[i-1] = Buzzer_indicate[i];  
        }
        Buzzer_indicate[Buzzer_indicate_Num - 1].Buzzer_Time = 0;
        Buzzer_indicate[Buzzer_indicate_Num - 1].Buzzer_Flicker = 0;
        Buzzer_indicate_Num --;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  vTaskDelete(NULL);
}