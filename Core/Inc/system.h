/**
 * @file system.h
 * @author boning@ucsb.edu
 * @brief This file includes the declarations of system level functions, flags and variables.
 * @version 0.1
 * @date 2020-05-17
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/**
 * @brief Initialize the interfaces of the microcontroller to avoid conflicts with the perpherals.
 * 
 */

#include "main.h"

#define POWER_MANAGE_GROUP    GPIOE
#define POWER_GOOD_PIN        GPIO_PIN_4
#define POWER_CHG_PIN         GPIO_PIN_2

#define GPIO_PIN_ESP32_NSS    GPIO_PIN_15
#define GPIO_PIN_ESP32_SCK    GPIO_PIN_10
#define GPIO_PIN_ESP32_MOSI   GPIO_PIN_11
#define GPIO_PIN_ESP32_MISO   GPIO_PIN_12


#define GPIO_PIN_ESP32_TX     GPIO_PIN_11
#define GPIO_PIN_ESP32_RX     GPIO_PIN_10
#define GPIO_GROUP_ESP32_UART GPIOB

#define GPIO_PIN_SWO          GPIO_PIN_3
#define GPIO_GROUP_SWO        GPIOA

void system_init_interfaces();