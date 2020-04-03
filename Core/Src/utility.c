#include "main.h"
#include "utility.h"
#include <stdlib.h>
#include <string.h>

void swo_init() {
    GPIO_InitTypeDef GpioInitStruct = {0};
    GpioInitStruct.Alternate = GPIO_AF0_SWJ;
    GpioInitStruct.Mode = GPIO_MODE_AF_PP;
    GpioInitStruct.Pin = GPIO_PIN_3;
    GpioInitStruct.Pull = GPIO_NOPULL;
    GpioInitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GpioInitStruct);
}

int _write(int file, char *ptr, int len)
{
  int i=0;
  uint8_t buffer[len];
  memcpy(buffer, ptr, len);
  for(i=0 ; i<len ; i++) 
    ITM_SendChar(buffer[i]);
  return len;
}
