#include "WS_DIN.h"

bool DIN_Flag[8] = {0};                   // DIN current status flag
uint8_t DIN_Data = 0;
bool Relay_Immediate_Enable = Relay_Immediate_Default;

bool DIN_Read_CH1(void){
  DIN_Flag[0] = digitalRead(DIN_PIN_CH1);
  if(DIN_Flag[0]){
    DIN_Data |= (1 << 0); 
    return 1;
  }
  else{
    DIN_Data &= (~(1 << 0)); 
    return 0;
  }
}
bool DIN_Read_CH2(void){
  DIN_Flag[1] = digitalRead(DIN_PIN_CH2);
  if(DIN_Flag[1]){
    DIN_Data |= (1 << 1); 
    return 1;
  }
  else{
    DIN_Data &= (~(1 << 1)); 
    return 0;
  }
}
bool DIN_Read_CH3(void){
  DIN_Flag[2] = digitalRead(DIN_PIN_CH3);
  if(DIN_Flag[2]){
    DIN_Data |= (1 << 2); 
    return 1;
  }
  else{
    DIN_Data &= (~(1 << 2)); 
    return 0;
  }
}
bool DIN_Read_CH4(void){
  DIN_Flag[3] = digitalRead(DIN_PIN_CH4);
  if(DIN_Flag[3]){
    DIN_Data |= (1 << 3); 
    return 1;
  }
  else{
    DIN_Data &= (~(1 << 3)); 
    return 0;
  }
}
bool DIN_Read_CH5(void){
  DIN_Flag[4] = digitalRead(DIN_PIN_CH5);
  if(DIN_Flag[4]){
    DIN_Data |= (1 << 4); 
    return 1;
  }
  else{
    DIN_Data &= (~(1 << 4)); 
    return 0;
  }
}
bool DIN_Read_CH6(void){
  DIN_Flag[5] = digitalRead(DIN_PIN_CH6);
  if(DIN_Flag[5]){
    DIN_Data |= (1 << 5); 
    return 1;
  }
  else{
    DIN_Data &= (~(1 << 5)); 
    return 0;
  }
}
bool DIN_Read_CH7(void){
  DIN_Flag[6] = digitalRead(DIN_PIN_CH7);
  if(DIN_Flag[6]){
    DIN_Data |= (1 << 6); 
    return 1;
  }
  else{
    DIN_Data &= (~(1 << 6)); 
    return 0;
  }
}
bool DIN_Read_CH8(void){
  DIN_Flag[7] = digitalRead(DIN_PIN_CH8);
  if(DIN_Flag[7]){
    DIN_Data |= (1 << 7); 
    return 1;
  }
  else{
    DIN_Data &= (~(1 << 7)); 
    return 0;
  }
}
uint8_t DIN_Read_CHxs(){
  DIN_Read_CH1();
  DIN_Read_CH2();
  DIN_Read_CH3();
  DIN_Read_CH4();
  DIN_Read_CH5();
  DIN_Read_CH6();
  DIN_Read_CH7();
  DIN_Read_CH8();
  return DIN_Data;
}

static uint8_t DIN_Data_Old = 0;
void DINTask(void *parameter) {
  while(1){
    if(Relay_Immediate_Enable){
      DIN_Read_CHxs();
      if(DIN_Data_Old != DIN_Data){
        if(DIN_Inverse_Enable)
          Relay_Immediate_CHxs(~DIN_Data , DIN_Mode);
        else
          Relay_Immediate_CHxs(DIN_Data , DIN_Mode);
        DIN_Data_Old = DIN_Data;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  vTaskDelete(NULL);
}

void DIN_Init(void)
{
  pinMode(DIN_PIN_CH1, INPUT_PULLUP);
  pinMode(DIN_PIN_CH2, INPUT_PULLUP);
  pinMode(DIN_PIN_CH3, INPUT_PULLUP);
  pinMode(DIN_PIN_CH4, INPUT_PULLUP);
  pinMode(DIN_PIN_CH5, INPUT_PULLUP);
  pinMode(DIN_PIN_CH6, INPUT_PULLUP);
  pinMode(DIN_PIN_CH7, INPUT_PULLUP);
  pinMode(DIN_PIN_CH8, INPUT_PULLUP);

  DIN_Read_CHxs();
  if(DIN_Inverse_Enable)
    DIN_Data_Old = 0xFF;
  else
    DIN_Data_Old = 0x00;
  xTaskCreatePinnedToCore(
    DINTask,    
    "DINTask",   
    4096,                
    NULL,                 
    4,                   
    NULL,                 
    0                   
  );
}
