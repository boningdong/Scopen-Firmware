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

/**
 * @brief Initialize the system interfaces to avoid conflicts with other chips.
 * 
 */
void system_init_interfaces() {
  // Set the ESP32 TX and RX connections to be open-drain on MCU.
  // Avoid the conflicts of ESP32 firmware uploading.
  GPIO_InitTypeDef gpio_init_struct;
  gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;
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


