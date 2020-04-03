#include "main.h"
#include "touch.h"
#include "stdio.h"
#include "stdlib.h"

system_flags_t system_flags;
trackpad_flags_t trackpad_flags;
event_flags_t events;
channel_status_t channels;

#define RDY_GROUP GPIOA
#define RDY_PIN GPIO_PIN_10
#define RDY_PIN_NUM 10

GPIO_TypeDef *ready_ports = RDY_GROUP;
EXTI_TypeDef *exti = EXTI;

/* static functions */
static void set_ready_output() {
  ready_ports->MODER |= 1UL << RDY_PIN_NUM * 2 ;
}

static void set_ready_input() {
  ready_ports->MODER &= ~(3UL << RDY_PIN_NUM * 2);
}

static void disable_ready_interrupt() {
  exti->IMR1 &= ~(1UL << RDY_PIN_NUM);
  exti->FTSR1 &= ~(1UL << RDY_PIN_NUM);
}

static void enable_ready_interrupt() {
  exti->IMR1 |= 1UL << RDY_PIN_NUM;
  exti->FTSR1 |= 1UL << RDY_PIN_NUM;
}

static void toggle_ready() {
  set_ready_output();
  HAL_GPIO_WritePin(RDY_GROUP, RDY_PIN, GPIO_PIN_RESET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(RDY_GROUP, RDY_PIN, GPIO_PIN_SET);
  set_ready_input();
}

static void read_random_bytes(uint16_t address, uint8_t buffer[], uint16_t size) {
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c3, TOUCH_ADDRESS << 1, address, 1, buffer, size, 200);
  if (status == HAL_ERROR)
    printf("read random bytes failed.");
}

static void write_random_bytes(uint16_t address, uint8_t buffer[], uint16_t size) {
  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c3, TOUCH_ADDRESS << 1, address, 1, buffer, size, 200);
  if (status == HAL_ERROR)
    printf("write random bytes failed.");
}

// driver functions
void clear_reset() {
  uint8_t buffer[2] = {0};
  read_random_bytes(PROXSETTINGS_23, buffer, 2);
  buffer[1] |= ACK_RESET_BIT;
  write_random_bytes(PROXSETTINGS_23, buffer, 2);
  printf("[reset] b1: %d, b2:%d\n", buffer[0], buffer[1]);
}

void read_device_info() {
  uint8_t buffer[2] = {0};
  read_random_bytes(DEVICE_INFO, buffer, 2);
  printf("product#: %d\nver#: %d\n", buffer[0], buffer[1]);
}

void read_events() {
  uint8_t buffer[2] = {0};
  read_random_bytes(EVENT_MASK, buffer, 2);
  printf("events: %d | zoom: %d\n", buffer[0], buffer[1]);
}

// init functions
void touch_init() {
  // init the ready pin
  __HAL_RCC_GPIOA_CLK_ENABLE();
  // configure as input mode for interrupt
  ready_ports->MODER &= ~(3UL << RDY_PIN_NUM * 2);     // input mode
  ready_ports->PUPDR |= 1UL << RDY_PIN_NUM * 2;      // pull-up
  // configure output parameters  
  ready_ports->OTYPER |= 1UL << RDY_PIN_NUM ;           // open drain
  ready_ports->OSPEEDR |= 3UL << (RDY_PIN_NUM * 2);    // very high speed
  ready_ports->BSRR |=  1UL << (RDY_PIN_NUM);           // set HIGH to be default state
  disable_ready_interrupt();
  // enable HAL_NVIC
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  
  
  if (request_comms())
    clear_reset();
  if (request_comms())
    read_device_info();
  if (request_comms())
    read_events();
}

bool request_comms() {
  bool response = false;
  uint16_t count = 0;
  // disable_ready_interrupt();
  // toggle ready
  toggle_ready();
  while(HAL_GPIO_ReadPin(RDY_GROUP, RDY_PIN) == GPIO_PIN_SET) {
    count ++;
    // Delay 1ms 
    HAL_Delay(1);
    if (count % 1000 == 0)
      return response;
    if (count % 100 == 0)
      toggle_ready();
  }
  response = true;
  // enable_ready_interrupt();
  return response;
}

// Used for blocking mode
bool wait_ready() {
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


//FIXME Delete this part for testing
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

void EXTI15_10_IRQHandler(void) {
  // Done something
  if (HAL_GPIO_ReadPin(RDY_GROUP, RDY_PIN) == GPIO_PIN_RESET) {
    printf("RDY pin low interrupt is triggered\n");
  }
}





