/**
 * @file i2c.c
 * @author boning@ucsb.edu
 * @brief Includes the low level drivers for the i2c perpherals.
 * @version 0.1
 * @date 2020-05-18
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "main.h"
#include "periph/i2c.h"
#include <stdio.h>
#include <stm32g4xx_ll_gpio.h>
#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_i2c.h>


ErrorStatus i2c3_init() {
  printf("[LOG] Initializing I2C3 ...\n");
  // Initialize GPIO
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  LL_GPIO_InitTypeDef io_config;
  io_config.Pin = LL_GPIO_PIN_8 | LL_GPIO_PIN_9;
  io_config.Mode = LL_GPIO_MODE_ALTERNATE;
  io_config.Alternate = LL_GPIO_AF_8;
  io_config.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  io_config.Pull = LL_GPIO_PULL_UP;
  io_config.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  if(LL_GPIO_Init(GPIOC, &io_config) != SUCCESS)
    return ERROR;

  // Initialize I2C3
  LL_AHB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C3);
  LL_I2C_InitTypeDef i2c_config;
  i2c_config.AnalogFilter = LL_I2C_ANALOGFILTER_ENABLE;
  i2c_config.DigitalFilter = 0;
  i2c_config.OwnAddress1 = 0;
  i2c_config.OwnAddrSize = LL_I2C_OWNADDRESS1_7BIT;
  i2c_config.PeripheralMode = LL_I2C_MODE_I2C;
  i2c_config.Timing = 0x10802D9B;                 // Calculated by cubemx for running at 400kHz.
  i2c_config.TypeAcknowledge = LL_I2C_ACK;
  if(LL_I2C_Init(I2C3, &i2c_config) != SUCCESS)
    return ERROR;

  LL_I2C_EnableAutoEndMode(I2C3);
  LL_I2C_EnableClockStretching(I2C3);
  LL_I2C_DisableOwnAddress2(I2C3);
  LL_I2C_DisableGeneralCall(I2C3);
  
  printf("[LOG] Successfully initialized I2C3.\n");
  return SUCCESS;
}


ErrorStatus i2c3_random_read(const uint16_t device, const uint8_t address, uint8_t* buffer, uint16_t size, bool restart) {
  if (buffer == NULL)
    return ERROR;

  // Timer used for timeout
  uint32_t timer = 0;
  // Address the register location
  // Use reload because the write sequence is not stoped.
  // Generate the start write signal to start the transmission.
  LL_I2C_HandleTransfer(I2C3, device << 1, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);
  
  // Make sure the tranmission register is empty.
  WAIT_CONDITION(LL_I2C_IsActiveFlag_TXIS(I2C3), 100, timer, return ERROR);
  LL_I2C_TransmitData8(I2C3, address);

  // Assert the tranmission is done.
  WAIT_CONDITION(LL_I2C_IsActiveFlag_TC(I2C3), 100, timer, return ERROR);
  // Start read condition
  uint32_t end_mode;
  end_mode = size > NBYTES_MAX ? LL_I2C_MODE_RELOAD : restart ? LL_I2C_MODE_SOFTEND : LL_I2C_MODE_AUTOEND;
  LL_I2C_HandleTransfer(I2C3, device << 1, LL_I2C_ADDRSLAVE_7BIT, size > NBYTES_MAX ? NBYTES_MAX : size, end_mode, LL_I2C_GENERATE_START_READ);
  // Receive payload
  do {
      while (size % (NBYTES_MAX + 1)) {
          // Receive data byte
          WAIT_CONDITION(LL_I2C_IsActiveFlag_RXNE(I2C3), 100, timer, return ERROR);
          *buffer++ = LL_I2C_ReceiveData8(I2C3);
          size--;
      }
      // Wait for bulk transaction completion if not generating a stop condition
      if (size) {
          WAIT_CONDITION(LL_I2C_IsActiveFlag_TCR(I2C3), 100, timer, return ERROR);
          // Reload NBYTES or last transaction and configure auto stop condition
          end_mode = size > NBYTES_MAX ? LL_I2C_MODE_RELOAD : restart ? LL_I2C_MODE_SOFTEND : LL_I2C_MODE_AUTOEND;
          LL_I2C_HandleTransfer(I2C3, device << 1, LL_I2C_ADDRSLAVE_7BIT, size > NBYTES_MAX ? NBYTES_MAX : size, end_mode, LL_I2C_GENERATE_NOSTARTSTOP);
      }
  } while (size);

  // Check if need to continue the transmission
  if (restart) {
    WAIT_CONDITION(LL_I2C_IsActiveFlag_TC(I2C3), 100, timer, return ERROR);
  } else {
    WAIT_CONDITION(LL_I2C_IsActiveFlag_STOP(I2C3), 100, timer, return ERROR);
    LL_I2C_ClearFlag_STOP(I2C3);
  }

  return SUCCESS;
}


ErrorStatus i2c3_random_write(const uint16_t device, const uint8_t address, const uint8_t* buffer, uint16_t size, bool restart) {
  if (buffer == NULL)
    return ERROR;
  
  // Timer used for timeout
  uint32_t timer = 0;
  // Address the register location
  // Use reload because the write sequence is not stoped.
  // Generate the start write signal to start the transmission.
  LL_I2C_HandleTransfer(I2C3, device << 1, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_RELOAD, LL_I2C_GENERATE_START_WRITE);

  // Make sure the tranmission register is empty.
  WAIT_CONDITION(LL_I2C_IsActiveFlag_TXIS(I2C3), DEFAULT_TIMEOUT, timer, return ERROR);
  LL_I2C_TransmitData8(I2C3, address);

  WAIT_CONDITION(LL_I2C_IsActiveFlag_TCR(I2C3), DEFAULT_TIMEOUT, timer, return ERROR);
  // Transmit the user data
  uint32_t end_mode;
  do {
    end_mode = size > NBYTES_MAX ? LL_I2C_MODE_RELOAD : restart ? LL_I2C_MODE_SOFTEND : LL_I2C_MODE_AUTOEND;
    LL_I2C_HandleTransfer(I2C3, device << 1, LL_I2C_ADDRSLAVE_7BIT, size > NBYTES_MAX ? NBYTES_MAX : size, end_mode, LL_I2C_GENERATE_NOSTARTSTOP);
    while (size % (NBYTES_MAX + 1)) {
      WAIT_CONDITION(LL_I2C_IsActiveFlag_TXIS(I2C3), DEFAULT_TIMEOUT, timer, return ERROR);
      LL_I2C_TransmitData8(I2C3, *buffer++);
      size --;
    }
    if (size) {
      WAIT_CONDITION(LL_I2C_IsActiveFlag_TCR(I2C3), DEFAULT_TIMEOUT, timer, return ERROR);
    }
  } while(size);

  // Check if need to continue the transmission
  if (restart) {
    WAIT_CONDITION(LL_I2C_IsActiveFlag_TC(I2C3), DEFAULT_TIMEOUT, timer, return ERROR);
  } else {
    WAIT_CONDITION(LL_I2C_IsActiveFlag_STOP(I2C3), DEFAULT_TIMEOUT, timer, return ERROR);
    LL_I2C_ClearFlag_STOP(I2C3);
  }

  return SUCCESS;
}