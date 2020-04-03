#ifndef __LED_H__
#define __LED_H__

#define GPIO_LED_GROUP GPIOC
#define GPIO_LED_RED_PIN GPIO_PIN_6
#define GPIO_LED_GREEN_PIN GPIO_PIN_7

void led_init();

#endif