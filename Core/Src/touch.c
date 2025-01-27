#include "touch.h"
#include "main.h"
#include "commands.h"
#include "cmsis_os.h"
#include "sthreads.h"
#include <stdio.h>
#include <stdlib.h>
#include "periph/iqs266.h"
#include <stm32g4xx_ll_bus.h>
#include <stm32g4xx_ll_exti.h>
#include <stm32g4xx_ll_gpio.h>


event_flags_t events = {0};
trackpad_flags_t gestures = {0};
TIM_HandleTypeDef htim5;
/**
 * @brief Initialize and configure the touch sensor on this board.
 * 
 */
void touch_init() {
  iqs266_init();

  // Must successfully request the communication window first.
  while(!iqs266_request_communication());

  iqs266_clear_reset(true);
  
  // Set low power and normal power parameters.
  iqs266_set_report_rate_lp(0, true);
  iqs266_set_report_rate_nm(6, true);

  // Set events
  iqs266_disable_halt_timeout(true);
  iqs266_event_mode(true);
  iqs266_disable_all_events(true);
  iqs266_enable_events(SWIPE_EVENT_BIT | TAP_EVENT_BIT, true);

  // Set zoom timeout
  iqs266_set_zoom_timeout(50, true);  
  // Disable proximate
  iqs266_disable_channel(CH0, true);

  // Set touch parameters
  iqs266_set_touch_base_value(3, true);
  iqs266_set_touch_ati_target(35, true);
  iqs266_set_touch_sensitivity(ALL, 25, true);
  // Set tap parameters.
  iqs266_set_tap_timeout(75, true);
  iqs266_set_tap_threshold(35, true);
  // Set swipe parameters
  iqs266_set_swipe_timeout(150, true);
  iqs266_set_swipe_threshold(40, true);
  // Set autotune.
  iqs266_auto_tune(false);
  // Delay 100ms to make sure the system is ready.
  printf("Done Initializing the touch sensor.\n");
  HAL_Delay(100);
  touch_init_timer();
  start_debounce_timer();
  iqs266_init_ready_interrupt();
}

void touch_init_timer(){
  __HAL_RCC_TIM5_CLK_ENABLE();
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = TIM_CLOCKPRESCALER_DIV2;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = (uint32_t)100000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
    HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

void stop_debounce_timer(){
  HAL_TIM_Base_Stop_IT(&htim5);
}

void start_debounce_timer(){
  HAL_TIM_Base_Start_IT(&htim5);
}


void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */
  stop_debounce_timer();
  /* USER CODE END TIM5_IRQn 1 */
}

/**
 * @brief Touch communication interrupt trigger. This will be triggered if there is an event.
 * 
 */
void EXTI15_10_IRQHandler(void) {
  // Check if this interrupt is caused by the touch sensor.
  printf("Touch int\r\n");
  if (HAL_GPIO_ReadPin(GPIO_RDY_GROUP, GPIO_RDY_PIN) == GPIO_PIN_RESET) {
    // Disable interrupt.
    iqs266_disable_rdy_interrupt();

    // Reading the flags to determine what triggers the interrput.
    events.flagByte = iqs266_read_events(true);
    gestures.flagByte = iqs266_read_gestures(false);
    
    if(events.tap) {
      if (gestures.tap) {
        latest_cmd.type = CMD_CHANGE_SEL;
        osSignalSet(event_handle_task, USER_INPUT_SIG);
      }
    } else if(events.swipe) {
      if (gestures.swipeUp) {
        // Note, here the up/down is determined by the touchpad direction.
        // Determine this value by testing.
        latest_cmd.type = CMD_SWIPE_DOWN;
        osSignalSet(event_handle_task, USER_INPUT_SIG);
      }
      else if (gestures.swipeDown) {
        // Note, here the up/down is determined by the touchpad direction.
        // Determine this value by testing.
        latest_cmd.type = CMD_SWIPE_UP;
        osSignalSet(event_handle_task, USER_INPUT_SIG);
      }
    }

    // Enable interrupt again.
    iqs266_enable_rdy_interrupt();
  }
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  printf("Touch int end\r\n");
}