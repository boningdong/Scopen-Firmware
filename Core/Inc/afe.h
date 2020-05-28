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
#define ADC_DATA_LENGTH 100

extern uint8_t adc1Data[ADC_DATA_LENGTH];
extern uint8_t adc2Data[ADC_DATA_LENGTH];
extern uint8_t adc4Data[ADC_DATA_LENGTH];
extern uint8_t adc5Data[ADC_DATA_LENGTH];
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