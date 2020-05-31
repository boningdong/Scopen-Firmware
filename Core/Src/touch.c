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
  iqs266_set_touch_sensitivity(ALL, 55, true);
  // Set tap parameters.
  iqs266_set_tap_timeout(75, true);
  iqs266_set_tap_threshold(35, true);
  // Set swipe parameters
  iqs266_set_swipe_timeout(150, true);
  iqs266_set_swipe_threshold(20, true);
  // Set autotune.
  iqs266_auto_tune(false);
  // Delay 100ms to make sure the system is ready.
  printf("Done Initializing the touch sensor.\n");
  HAL_Delay(100);
  iqs266_init_ready_interrupt();
}

/**
 * @brief Touch communication interrupt trigger. This will be triggered if there is an event.
 * 
 */
void EXTI15_10_IRQHandler(void) {
  // Check if this interrupt is caused by the touch sensor.
  if (HAL_GPIO_ReadPin(GPIO_RDY_GROUP, GPIO_RDY_PIN) == GPIO_PIN_RESET) {
    // Disable interrupt.
    iqs266_disable_rdy_interrupt();

    // Reading the flags to determine what triggers the interrput.
    events.flagByte = iqs266_read_events(true);
    gestures.flagByte = iqs266_read_gestures(false);
    
    if(events.tap) {
      if (gestures.tap) {
        latest_cmd.type = CMD_CHANGE_SEL;
        latest_cmd.argv = NULL;
        latest_cmd.argc = 0;
        osSignalSet(event_handle_task, USER_INPUT_SIG);
      }
    } else if(events.swipe) {
      if (gestures.swipeUp) {
        latest_cmd.type = CMD_SWIPE_UP;
        latest_cmd.argv = NULL;
        latest_cmd.argc = 0;
        osSignalSet(event_handle_task, USER_INPUT_SIG);
      }
      else if (gestures.swipeDown) {
        latest_cmd.type = CMD_SWIPE_DOWN;
        latest_cmd.argv = NULL;
        latest_cmd.argc = 0;
        osSignalSet(event_handle_task, USER_INPUT_SIG);
      }
    }

    // Enable interrupt again.
    iqs266_enable_rdy_interrupt();
  }
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
}