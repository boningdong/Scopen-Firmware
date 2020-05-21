#include "touch.h"


void touch_init() {
  iqs266_init();

  // Must successfully request the communication window first.
  while(!iqs266_request_communication());
  iqs266_clear_reset(true);
  
  // Set events
  iqs266_enable_all_events(true);
  iqs266_enable_events(SWIPE_EVENT_BIT | TAP_EVENT_BIT, true);

  // Set zoom timeout
  // Set touch parameters
  // Set autotune.
  iqs266_auto_tune(false);
  
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
// void EXTI15_10_IRQHandler(void) {
//   // Done something
//   if (HAL_GPIO_ReadPin(RDY_GROUP, RDY_PIN) == GPIO_PIN_RESET) {
//     printf("RDY pin low interrupt is triggered\n");
//   }
// }