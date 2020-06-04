/**
 * @file system.c
 * @author boning@ucsb.edu
 * @brief This file includes the definitions of system level functions and variables.
 * @version 0.1
 * @date 2020-05-17
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "system.h"
#include <stdio.h>
#include "led.h"
#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_i2c.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_exti.h>

/**
 * @brief Initialize the system interfaces to avoid conflicts with other chips.
 * 
 */
void system_init_interfaces() {
  // Set the ESP32 TX and RX connections to be open-drain on MCU.
  // Avoid the conflicts of ESP32 firmware uploading.
  GPIO_InitTypeDef gpio_init_struct;
  gpio_init_struct.Mode = GPIO_MODE_INPUT;
  gpio_init_struct.Pin = GPIO_PIN_ESP32_TX | GPIO_PIN_ESP32_RX;
  gpio_init_struct.Pull = GPIO_PULLUP;
  gpio_init_struct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIO_GROUP_ESP32_UART, &gpio_init_struct);
  
  // Enable the SWO debug interface.
  gpio_init_struct.Mode = GPIO_MODE_AF_PP;
  gpio_init_struct.Alternate = GPIO_AF0_TRACE;
  gpio_init_struct.Pin = GPIO_PIN_SWO;
  gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIO_GROUP_SWO, &gpio_init_struct);

  // REVIEW: Here I was trying to do the charge trigger. Falling and rising mode.
  // CHG_PIN Should be PE2
  __HAL_RCC_GPIOE_CLK_ENABLE();
  gpio_init_struct.Mode = GPIO_MODE_IT_RISING_FALLING ;
  gpio_init_struct.Pin = POWER_CHG_PIN;
  gpio_init_struct.Pull = GPIO_PULLUP;
  gpio_init_struct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(POWER_MANAGE_GROUP, &gpio_init_struct);


  // Enable the power charging pin.
  // LL_EXTI_InitTypeDef exti_config = {0};
  // exti_config.Line_0_31 = LL_EXTI_LINE_2;
  // exti_config.LineCommand = ENABLE;
  // exti_config.Mode = LL_EXTI_MODE_IT;
  // exti_config.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
  // LL_EXTI_Init(&exti_config);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 8, 8);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

/**
 * @brief Override the weak function _write to hook the printf output to SWO.
 * 
 * @param file: file descriptor.
 * @param ptr: message buffer.
 * @param len: message length.
 * @return int: numbers that were written. 
 */
int _write(int file, char *ptr, int len) {
  for (int i = 0; i < len; i++)
    ITM_SendChar((*ptr++));
  return len;
}

/**
 * @brief This handler will handle the charging interrupt.
 * 
 */
void EXTI2_IRQHandler(void) {
  if (HAL_GPIO_ReadPin(POWER_MANAGE_GROUP, POWER_CHG_PIN) == GPIO_PIN_RESET) {
    HAL_GPIO_WritePin(GPIO_LED_GROUP, GPIO_LED_GREEN_PIN, GPIO_PIN_SET);
  } else if (HAL_GPIO_ReadPin(POWER_MANAGE_GROUP, POWER_CHG_PIN) == GPIO_PIN_SET) {
    HAL_GPIO_WritePin(GPIO_LED_GROUP, GPIO_LED_GREEN_PIN, GPIO_PIN_RESET);
  }
  HAL_GPIO_EXTI_IRQHandler(POWER_CHG_PIN);
}


