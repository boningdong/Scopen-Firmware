/**
 * @file afe.h
 * @author boning@ucsb.edu baguilar@ucsb.edu
 * @brief Includes all of the prototypes related to the AFE.
 * @version 0.1
 * @date 2020-05-17
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __AFE_H__
#define __AFE_H__

#include "main.h"

typedef struct {
  uint8_t speed_option;
  uint32_t sample_length;
} sample_paras_t;


extern sample_paras_t sample_paras;

/**
 * @brief Sampling Speed Options (indices).
 * 
 */
typedef enum {
  SAMPLE_SPEED_HIGHTEST,
  SAMPLE_SPEED_HIGH,
  SAMPLE_SPEED_MEDIUM,
  SAMPLE_SPEED_LOW,
  SAMPLE_SPEED_LOWEST
} sample_speed_t;

/**
 * @brief Sampling configuration struct.
 * Records the parameters that are used for setting up the HRTIM.
 */
typedef struct {
  uint32_t timer_prescaler;
  uint32_t timer_period;
} sample_config_t;

// extern uint8_t adc1Data[ADC_DATA_LENGTH];
// extern uint8_t adc2Data[ADC_DATA_LENGTH];
// extern uint8_t adc4Data[ADC_DATA_LENGTH];
// extern uint8_t adc5Data[ADC_DATA_LENGTH];
void afe_initialize();
void afe_adc_initialize();
void afe_adc_hrtim_initialize();
void afe_sampling_start();
void afe_sampling_stop();
void afe_relay_control(bool on);
void DMA1_Channel1_IRQHandler(void);
void DMA2_Channel1_IRQHandler(void);
void DMA1_Channel2_IRQHandler(void);
void DMA2_Channel2_IRQHandler(void);

#endif