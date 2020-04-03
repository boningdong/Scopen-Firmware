#include "led.h"
#include "main.h"

void led_init() {
  GPIO_InitTypeDef gpioInitStruct = {0};
  gpioInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  gpioInitStruct.Pin = GPIO_LED_RED_PIN | GPIO_LED_GREEN_PIN;
  gpioInitStruct.Pull = GPIO_PULLDOWN;
  gpioInitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIO_LED_GROUP, &gpioInitStruct);
  HAL_GPIO_WritePin(GPIO_LED_GROUP, GPIO_LED_RED_PIN | GPIO_LED_GREEN_PIN, GPIO_PIN_RESET);
}