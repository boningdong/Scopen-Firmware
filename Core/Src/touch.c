#include "main.h"
#include "touch.h"
#include "stdio.h"
#include "stdlib.h"
#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_gpio.h>



system_flags_t system_flags;
trackpad_flags_t trackpad_flags;
event_flags_t events;
channel_status_t channels;

#define RDY_GROUP GPIOA
#define RDY_PIN GPIO_PIN_10
#define RDY_PIN_NUM 10

GPIO_TypeDef *ready_ports = RDY_GROUP;
EXTI_TypeDef *exti = EXTI;

// driver functions
void clear_reset() {
  uint8_t buffer[2] = {0};
  _read_random_bytes(PROXSETTINGS_23, buffer, 2);
  buffer[1] |= ACK_RESET_BIT;
  _write_random_bytes(PROXSETTINGS_23, buffer, 2);
  printf("[reset] b1: %d, b2:%d\n", buffer[0], buffer[1]);
}

void read_device_info() {
  uint8_t buffer[2] = {0};
  _read_random_bytes(DEVICE_INFO, buffer, 2);
  printf("product#: %d\nver#: %d\n", buffer[0], buffer[1]);
}

void read_events() {
  uint8_t buffer[2] = {0};
  _read_random_bytes(EVENT_MASK, buffer, 2);
  printf("events: %d | zoom: %d\n", buffer[0], buffer[1]);
}

// init functions
void iqs266_init() {
  // Initialize the RDY pin
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

  LL_GPIO_InitTypeDef io_config;             
  io_config.Pin = TOUCH_RDY_PIN;
  io_config.Mode = LL_GPIO_MODE_INPUT;
  io_config.Pull = LL_GPIO_PULL_UP;
  io_config.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  io_config.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  LL_GPIO_Init(TOUCH_RDY_GROUP, &io_config);

  _disable_ready_interrupt();
  
  // enable HAL_NVIC
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  
  
  if (iqs266_request_comms())
    clear_reset();
  if (iqs266_request_comms())
    read_device_info();
  if (iqs266_request_comms())
    read_events();
}

bool iqs266_request_comms() {
  bool response = false;
  uint16_t count = 0;
  // disable_ready_interrupt();
  // toggle ready
  _toggle_ready();
  while(HAL_GPIO_ReadPin(RDY_GROUP, RDY_PIN) == GPIO_PIN_SET) {
    count ++;
    // Delay 1ms 
    HAL_Delay(1);
    if (count % 1000 == 0)
      return response;
    if (count % 100 == 0)
      _toggle_ready();
  }
  response = true;
  // enable_ready_interrupt();
  return response;
}

// Used for blocking mode
bool iqs266_wait_ready() {
  bool ready = false;
  uint16_t count = 0;
  while(HAL_GPIO_ReadPin(RDY_GROUP, RDY_PIN) == GPIO_PIN_SET) {
    count ++;
    HAL_Delay(1);
    if(count % 1000 == 0)
      return ready;
  }
  ready = true;
  return ready;
}


/* Some interrupt notes here */
/* There are 16 interrupt lines line0 - line15 (EXTI lines)
 * These lines are corresponding to pin0 - pin15 in each GPIO group
 * All pinX connected to lineX a multiplexed manner
 * Corresponding line will trigger corresponding EXTI_IRQ function
 * ex: PA1 interrupt will trigger EXTI1_IRQHandler function.
 */


// FIXME Delete this part for testing
void user_button_init() {
  // configure the user button to be interrupt mode.
  // user button pc13

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  // Hal method to configure the gpio
  // GPIO_InitTypeDef gpioConfig = {0};
  // gpioConfig.Mode = GPIO_MODE_IT_RISING;
  // gpioConfig.Pull = GPIO_NOPULL;
  // gpioConfig.Speed = GPIO_SPEED_MEDIUM;
  // gpioConfig.Pin = GPIO_PIN_13;
  // HAL_GPIO_Init(GPIOC, &gpioConfig);

  GPIO_InitTypeDef ledConfig = {0};
  ledConfig.Mode = GPIO_MODE_OUTPUT_PP;
  ledConfig.Pull = GPIO_PULLDOWN;
  ledConfig.Speed = GPIO_SPEED_MEDIUM;
  ledConfig.Pin = GPIO_PIN_5;
  HAL_GPIO_Init(GPIOA, &ledConfig);

  // configure GPIO port
  GPIO_TypeDef *gpioc = GPIOC;
  gpioc->MODER &= ~(GPIO_MODER_MODE13_0 | GPIO_MODER_MODE13_1);
  gpioc->PUPDR &= ~(GPIO_PUPDR_PUPDR13_0 | GPIO_PUPDR_PUPDR13_1);
  // configure interrupt mode
  EXTI_TypeDef *exti = EXTI;
  // Disable the mask (mask will mask out the interrupt trigger)
  exti->IMR1 |= EXTI_IMR1_IM13;
  // Configure the rising edge trigger.
  exti->RTSR1 |= EXTI_RTSR1_RT13;
  // Enable the correponding interrupt in NVIC
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
 * @brief Touch communication interrupt trigger. This will be triggered if there is an event.
 * 
 */
void EXTI15_10_IRQHandler(void) {
  // Done something
  if (HAL_GPIO_ReadPin(RDY_GROUP, RDY_PIN) == GPIO_PIN_RESET) {
    printf("RDY pin low interrupt is triggered\n");
  }
}




// Static functions definitions
static void _set_ready_output() {
  ready_ports->MODER |= 1UL << RDY_PIN_NUM * 2 ;
}

static void _set_ready_input() {
  ready_ports->MODER &= ~(3UL << RDY_PIN_NUM * 2);
}

static void _disable_ready_interrupt() {
  exti->IMR1 &= ~(1UL << RDY_PIN_NUM);
  exti->FTSR1 &= ~(1UL << RDY_PIN_NUM);
}

static void _enable_ready_interrupt() {
  exti->IMR1 |= 1UL << RDY_PIN_NUM;
  exti->FTSR1 |= 1UL << RDY_PIN_NUM;
}

/**
 * @brief Generate a falling edge to request a communication window.
 * 
 */
static void _toggle_ready() {
  _set_ready_output();
  HAL_GPIO_WritePin(RDY_GROUP, RDY_PIN, GPIO_PIN_RESET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(RDY_GROUP, RDY_PIN, GPIO_PIN_SET);
  _set_ready_input();
}

static void _read_random_bytes(uint16_t address, uint8_t buffer[], uint16_t size) {
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c3, TOUCH_ADDRESS << 1, address, 1, buffer, size, 200);
  if (status == HAL_ERROR)
    printf("read random bytes failed.");
}

static void _write_random_bytes(uint16_t address, uint8_t buffer[], uint16_t size) {
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c3, TOUCH_ADDRESS << 1, address, 1, buffer, size, 200);
  if (status == HAL_ERROR)
    printf("write random bytes failed.");
}





