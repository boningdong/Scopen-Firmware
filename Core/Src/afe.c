/**
 * @file afe.c
 * @author boning@ucsb.edu baguilar@ucsb.edu cesar04@ucsb.edu
 * @brief Includes all of the prototypes related to the AFE.
 * @version 0.1
 * @date 2020-05-17
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/**
 * @brief Interface and configuration initialization.
 * 
 */

#include "main.h"
#include "afe.h"
#include <math.h>
#include "sram.h"
#include "communication.h"
#include "sthreads.h"
#include "stm32g4xx_ll_hrtim.h"
#include "stm32g4xx_ll_dma.h"
#include "cmsis_os.h"
#include <stdio.h>

#define ADC1_GPIO GPIO_PIN_0|GPIO_PIN_1
#define ADC2_GPIO GPIO_PIN_6|GPIO_PIN_7
#define ADC4_GPIO GPIO_PIN_12|GPIO_PIN_14;
#define ADC5_GPIO GPIO_PIN_8|GPIO_PIN_9

/**
 * @note Here if the ADC_BUFFER_BASE uses the SRAM_BANK_ADDRESS, which is the external SRAM.
 * - In communication.c, should use hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD.
 * - Also all the ADC DMAs should have MemDataAlignment = DMA_MDATAALIGN_HALFWORD.
 * - The ADC Buffer should have uint16_t.
 */ 

#define ADC_SAMPLE_LENGTH     (sample_paras.sample_length)
#define ADC_BUFFER_BASE       SRAM_BANK_ADDRESS
#define ADC_BUFFER_A          ((uint32_t*)(ADC_BUFFER_BASE + HEADER_SIZE * sizeof(uint16_t)))
#define ADC_BUFFER_B          ((uint32_t*)(ADC_BUFFER_BASE + HEADER_SIZE * sizeof(uint16_t) + sample_paras.sample_length/4 * 1 * sizeof(uint16_t)))
#define ADC_BUFFER_C          ((uint32_t*)(ADC_BUFFER_BASE + HEADER_SIZE * sizeof(uint16_t) + sample_paras.sample_length/4 * 2 * sizeof(uint16_t)))
#define ADC_BUFFER_D          ((uint32_t*)(ADC_BUFFER_BASE + HEADER_SIZE * sizeof(uint16_t) + sample_paras.sample_length/4 * 3 * sizeof(uint16_t)))

/**
 * @brief Global switch to control whether the sampling is enabled.
 * 
 */
// osMutexId mutex_sample_ctrl;
osSemaphoreId sem_sample_ctrl;
bool sampling_enabled = false;

/**
 * @brief Records the sampling length of the last conversion. 
 * This vairable is needed because when sending the data, it needs to know how many data it converted.
 * @note this variable should only be modifed in afe_sampling_trigger (before the conversion starts).
 */
uint32_t last_conv_length;

/**
 * @brief Sample parameters used by the current ADC.
 * Changing this parameters and call corresponding function to load this parameters into sampling config.
 * 
 */
sample_paras_t sample_paras = {0};
/**
 * @brief Sample configurations for five speed levels.
 * @note  These values should be selected to make the hrtim and ADC run at the performance that the Scopen-App expects. 
 *        These values are calculated based on the system frequency (input to HRTIM1) to be 170MHz. Make sure the system frequency is correct.  
 */
osSemaphoreId sem_sample_paras;
const sample_config_t sample_configs[] = {
  {.timer_prescaler = HRTIM_PRESCALERRATIO_MUL32,  .timer_period = 362},        // 15MHz Sampling Speed
  {.timer_prescaler = HRTIM_PRESCALERRATIO_MUL32,  .timer_period = 5434},       // 1MHz Sampling Speed   
  {.timer_prescaler = HRTIM_PRESCALERRATIO_MUL8,  .timer_period = 13605},       // 100KHz Sampling Speed
  {.timer_prescaler = HRTIM_PRESCALERRATIO_DIV2,  .timer_period = 8503},       // 10KHz Sampling Speed
  {.timer_prescaler = HRTIM_PRESCALERRATIO_DIV2,  .timer_period = 8503}        // 10KHz Sampling Speed
};

DAC_HandleTypeDef hdac1;
DAC_HandleTypeDef hdac3;

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc4;
ADC_HandleTypeDef hadc5;

DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc4;
DMA_HandleTypeDef hdma_adc5;

HRTIM_HandleTypeDef hhrtim1;

OPAMP_HandleTypeDef hopamp3;

// uint8_t adc1Data[ADC_DATA_LENGTH];
// uint8_t adc2Data[ADC_DATA_LENGTH];
// uint8_t adc4Data[ADC_DATA_LENGTH];
// uint8_t adc5Data[ADC_DATA_LENGTH];

/**
 * @brief Initializes all AFE peripherals
 *        Uses afe_adc_initialize and afe_adc_hrtim_initialize
 */
void afe_initialize(){
    // Initialize the global configs and mutex.
    osSemaphoreDef(SampleSwitch);
    osSemaphoreDef(ParasLock);
    sem_sample_ctrl = osSemaphoreCreate(osSemaphore(SampleSwitch), 1);
    sem_sample_paras = osSemaphoreCreate(osSemaphore(ParasLock), 1);
    

    //Initializes Relay pin and sets it to off
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);   

    //DAC for gain control in VGA
    __HAL_RCC_DAC1_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    DAC_ChannelConfTypeDef sConfig = {0};
    hdac1.Instance = DAC1;
    if (HAL_DAC_Init(&hdac1) != HAL_OK)
    {
        Error_Handler();
    }
    sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
    sConfig.DAC_DMADoubleDataMode = DISABLE;
    sConfig.DAC_SignedFormat = DISABLE;
    sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    //DAC for VCM control in DIFF AMP
    //Enables opamp buffer first then the DAC peripheral
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_DAC3_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    hopamp3.Instance = OPAMP3;
    hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
    hopamp3.Init.Mode = OPAMP_FOLLOWER_MODE;
    hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_DAC;
    hopamp3.Init.InternalOutput = DISABLE;
    hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
    hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
    if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
    {
        Error_Handler();
    }

    hdac3.Instance = DAC3;
    if (HAL_DAC_Init(&hdac3) != HAL_OK)
    {
        Error_Handler();
    }
    sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
    sConfig.DAC_DMADoubleDataMode = DISABLE;
    sConfig.DAC_SignedFormat = DISABLE;
    sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
    sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 8, 8);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    /* DMA1_Channel2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 8, 8);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    /* DMA2_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 8, 8);
    HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
    /* DMA2_Channel2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 8, 8);
    HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
    afe_adc_initialize();
    afe_adc_hrtim_initialize();
    HAL_OPAMP_Start(&hopamp3);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_Start(&hdac3, DAC_CHANNEL_2);
}

void afe_adc_initialize() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    //ADC1
    __HAL_RCC_GPIOA_CLK_ENABLE();
     __HAL_RCC_ADC12_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = ADC1_GPIO;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hdma_adc1.Instance = DMA1_Channel1; //DMA for adc data
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_NORMAL;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_LINKDMA(&hadc1,DMA_Handle,hdma_adc1);
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};
    
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc1.Init.Resolution = ADC_RESOLUTION_8B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.GainCompensation = 0;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_HRTIM_TRG1;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
    Error_Handler();
    }
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
    {
    Error_Handler();
    }
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
    Error_Handler();
    }

    //ADC2
    GPIO_InitStruct.Pin = ADC2_GPIO;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hdma_adc2.Instance = DMA2_Channel1;
    hdma_adc2.Init.Request = DMA_REQUEST_ADC2;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_NORMAL;
    hdma_adc2.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&hadc2,DMA_Handle,hdma_adc2);

    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc2.Init.Resolution = ADC_RESOLUTION_8B;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.GainCompensation = 0;
    hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc2.Init.LowPowerAutoWait = DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.NbrOfConversion = 1;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_HRTIM_TRG3;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc2.Init.DMAContinuousRequests = ENABLE;
    hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc2.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc2) != HAL_OK)
    {
        Error_Handler();
    }
    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    //ADC4
    __HAL_RCC_ADC345_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = ADC4_GPIO;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hdma_adc4.Instance = DMA1_Channel2;
    hdma_adc4.Init.Request = DMA_REQUEST_ADC4;
    hdma_adc4.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc4.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc4.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc4.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc4.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc4.Init.Mode = DMA_NORMAL;
    hdma_adc4.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc4) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&hadc4,DMA_Handle,hdma_adc4);

    hadc4.Instance = ADC4;
    hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc4.Init.Resolution = ADC_RESOLUTION_8B;
    hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc4.Init.GainCompensation = 0;
    hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc4.Init.LowPowerAutoWait = DISABLE;
    hadc4.Init.ContinuousConvMode = DISABLE;
    hadc4.Init.NbrOfConversion = 1;
    hadc4.Init.DiscontinuousConvMode = DISABLE;
    hadc4.Init.ExternalTrigConv = ADC_EXTERNALTRIG_HRTIM_TRG2;
    hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc4.Init.DMAContinuousRequests = ENABLE;
    hadc4.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc4.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc4) != HAL_OK)
    {
        Error_Handler();
    }
    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    //ADC5
    GPIO_InitStruct.Pin = ADC5_GPIO;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hdma_adc5.Instance = DMA2_Channel2;
    hdma_adc5.Init.Request = DMA_REQUEST_ADC5;
    hdma_adc5.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc5.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc5.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc5.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc5.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc5.Init.Mode = DMA_NORMAL;
    hdma_adc5.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_adc5) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(&hadc5,DMA_Handle,hdma_adc5);

    hadc5.Instance = ADC5;
    hadc5.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc5.Init.Resolution = ADC_RESOLUTION_8B;
    hadc5.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc5.Init.GainCompensation = 0;
    hadc5.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc5.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc5.Init.LowPowerAutoWait = DISABLE;
    hadc5.Init.ContinuousConvMode = DISABLE;
    hadc5.Init.NbrOfConversion = 1;
    hadc5.Init.DiscontinuousConvMode = DISABLE;
    hadc5.Init.ExternalTrigConv = ADC_EXTERNALTRIG_HRTIM_TRG4;
    hadc5.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc5.Init.DMAContinuousRequests = ENABLE;
    hadc5.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc5.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc5) != HAL_OK)
    {
        Error_Handler();
    }
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff =ADC_DIFFERENTIAL_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc5, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

void afe_adc_hrtim_initialize(void)
{
  __HAL_RCC_HRTIM1_CLK_ENABLE();
  uint32_t period = 4*4;
  HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg = {0};
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_CompareCfgTypeDef pCompareCfg = {0};

  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
  {
    Error_Handler();
  }
  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_MASTER;
  // ADC triggers on master compared value 1.
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT13_MASTER_CMP1;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, &pADCTriggerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, 0x0) != HAL_OK)
  {
    Error_Handler();
  }
  // ADC triggers on master compared value 3.
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT24_MASTER_CMP3;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_2, &pADCTriggerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_2, 0x0) != HAL_OK)
  {
    Error_Handler();
  }
  // ADC triggers on master compared value 2.
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT13_MASTER_CMP2;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_3, &pADCTriggerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_3, 0x0) != HAL_OK)
  {
    Error_Handler();
  }
  // ADC triggers on master period is reached.
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT24_MASTER_PERIOD;
  if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_4, &pADCTriggerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_ADCPostScalerConfig(&hhrtim1, HRTIM_ADCTRIGGER_4, 0x0) != HAL_OK)
  {
    Error_Handler();
  }
  // Here set the master period time.
  pTimeBaseCfg.Period = period;
  // The repetition counter means after how many counter (reset) events, the interrupt will be triggered.
  pTimeBaseCfg.RepetitionCounter = 64;
  // The prescaler value determines timer frequency.
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_DIV1;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_MASTER_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_MASTER_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  pTimerCfg.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;

  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  // Here configures the 3 value compare registers.
  pCompareCfg.CompareValue = period/4;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = period/2;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pCompareCfg.CompareValue = period*3/4;
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, &pCompareCfg) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
 * @brief Trigger sampling. This function will recalibrate the ADCs first and then start.
 * Sampling length will be configured inside this function.
 */
void afe_sampling_trigger() {
  // Lock the sampling parameters first.
  osSemaphoreWait(sem_sample_paras, osWaitForever);
  //calibrates all ADCs and starts them in DMA mode
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_DIFFERENTIAL_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc4, ADC_DIFFERENTIAL_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc5, ADC_DIFFERENTIAL_ENDED);
  HAL_ADC_Start_DMA(&hadc1, ADC_BUFFER_A, ADC_SAMPLE_LENGTH / 4);
  HAL_ADC_Start_DMA(&hadc2, ADC_BUFFER_B, ADC_SAMPLE_LENGTH / 4);
  HAL_ADC_Start_DMA(&hadc4, ADC_BUFFER_C, ADC_SAMPLE_LENGTH / 4);
  HAL_ADC_Start_DMA(&hadc5, ADC_BUFFER_D, ADC_SAMPLE_LENGTH / 4);
  HAL_HRTIM_SimpleBaseStart (&hhrtim1, HRTIM_TIMERINDEX_MASTER);
  last_conv_length = ADC_SAMPLE_LENGTH;
  osSemaphoreRelease(sem_sample_paras);
}

/**
 * @brief Pause the hrtim to stop triggering the functions.
 * 
 */
void afe_sampling_pause() {
  HAL_HRTIM_SimpleBaseStop(&hhrtim1, HRTIM_TIMERINDEX_MASTER);
}

/**
 * @brief Call this function to set the pen into the sample mode.
 * @note  Use this function instread of setting the sampling_enabled directly to avoid race condition.
 */
void afe_sampling_enable() {
  osSemaphoreWait(sem_sample_ctrl, osWaitForever);
  sampling_enabled = true;
  osSemaphoreRelease(sem_sample_ctrl);
}

/**
 * @brief Call this function to set the pen into stop mode.
 * @note  Use this function instread of setting the sampling_enabled directly to avoid race condition.
 */
void afe_sampling_disable() {
  osSemaphoreWait(sem_sample_ctrl, osWaitForever);
  sampling_enabled = false;
  osSemaphoreRelease(sem_sample_ctrl);
}

/**
 * @brief Return if the hrtim is stopped so that it will not trigger the adc sampling.
 * 
 * @return true   The hrtim is stopped. No sampling will be triggered.
 * @return false  The hrtim is running. Sampling is still happening.
 */
bool afe_is_sampling_paused() {
  if(LL_HRTIM_TIM_IsCounterEnabled(HRTIM1, LL_HRTIM_TIMER_MASTER))
    return false;
  return true;
}

/**
 * @brief Check if the global switch is on or off.
 * 
 * @return true   The sampling is enabled.
 * @return false  The sampling is disabled.
 */
bool afe_is_sampling_enabled() {
  bool enabled;
  osSemaphoreWait(sem_sample_ctrl, osWaitForever);
  enabled = sampling_enabled;
  osSemaphoreRelease(sem_sample_ctrl);
  return enabled;
}

/**
 * @brief Configure the sampling length and speed using this function.
 * 
 * @param index   The sampling speed index. The detailed sampling speed levels are predefined values.
 * @param length  Sampling length determines after how many samples the pen should transmit the data to the software.
 * @note  This function need to be called when the timer is off.
 */
void afe_set_sampling_paras(uint8_t index, uint32_t length) {
  // Update the global sample parameters configuration.
  printf("[SET SAMPLE] Locking sample parameters...\r\n");
  osSemaphoreWait(sem_sample_paras, osWaitForever);
  sample_paras.sample_length = length;
  sample_paras.speed_option = index;
  
  HRTIM_CompareCfgTypeDef compareCfg = {0};
  HRTIM_TimeBaseCfgTypeDef timeCfg = {0};
  
  uint32_t sample_period = sample_configs[index].timer_period;
  uint32_t scaler = sample_configs[index].timer_prescaler;
  
  // There are 4 ADCs, so the total period should be multiply the sample period by 4.
  timeCfg.Period = sample_period * 4;
  timeCfg.PrescalerRatio = scaler;
  // Use continuous mode. Reload the counter after period overflows.
  timeCfg.Mode = HRTIM_MODE_CONTINUOUS;
  // This value doesn't matter. Because we are not using interrupt.
  timeCfg.RepetitionCounter = 64;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &timeCfg) != HAL_OK) {
    Error_Handler();
  }
  
  // Here configures the 3 value compare registers.
  compareCfg.CompareValue = (uint32_t)(sample_period);
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_1, &compareCfg) != HAL_OK) {
    Error_Handler();
  }
  compareCfg.CompareValue = (uint32_t)(sample_period * 2);
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_2, &compareCfg) != HAL_OK) {
    Error_Handler();
  }
  compareCfg.CompareValue = (uint32_t)(sample_period * 3);
  if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, HRTIM_COMPAREUNIT_3, &compareCfg) != HAL_OK) {
    Error_Handler();
  }
  osSemaphoreRelease(sem_sample_paras);
  printf("[SET SAMPLE] Unlocked sample parameters...\r\n");
}

/**
 * @brief Get the current sampling parameters including the sampling length and sampling speed index.
 * 
 * @param index   A uint8_t* that points to the buffer to be filled.
 * @param length  A uint32_t* that points to the buffer to be filled.
 */
void afe_get_current_sampling_paras(uint8_t* index, uint32_t* length) {
  if (index == NULL || length == NULL)
    return;
  osSemaphoreWait(sem_sample_paras, osWaitForever);
  *index = sample_paras.speed_option;
  *length = sample_paras.sample_length;
  osSemaphoreRelease(sem_sample_paras);
}

void afe_relay_control(bool on) {
    if(on){
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    }
    else{
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    }
}

void afe_set_gain(uint8_t mode) {
  float gaindb;
  float vga_in;
  float dac_output_analog;
  uint32_t dac_output_digitized;
  float pkpk = 1.8;
  float attenuated; 
  if(mode <= 4){
    afe_relay_control(1);
  }
  else{
    afe_relay_control(0);
  }
  switch(mode) {
      case 1: //10mV/Division 100mVpp
        gaindb = 20*log10(pkpk / 0.1);  
        break; 
      case 2: //20mV/Division 200mVpp
        gaindb = 20*log10(pkpk / 0.20);
        break; 
      case 3: //50mV/Division 500mVpp
        gaindb = 20*log10(pkpk / 0.5);
        break; 
      case 4: //100mV/Division 1Vpp
        gaindb = 20*log10(pkpk / 1.0);
        break; 
      case 5: //200mV/Division 2Vpp
        gaindb = 20*log10(pkpk / 2.0);
        break; 
      case 6: //500mV/Division 5Vpp                                                                                                                                                                                                                                                                                                             
        attenuated = 5 * 0.024; 
        gaindb = 20*log10(pkpk / attenuated);
        break; 
      case 7: //1V/Division 10vpp
       attenuated = 10 * 0.024; 
        gaindb = 20*log10(pkpk / attenuated);;
        break; 
      case 8: //2V/Division 20Vpp
        attenuated = 20 * 0.024; 
        gaindb = 20*log10(pkpk / attenuated);
        break; 
      case 9: //5V/Division 50Vpp
        attenuated = 50 * 0.024; 
        gaindb = 20*log10(pkpk / attenuated);
        break;
      case 10: //10V/Division 100Vpp
        attenuated = 100 * 0.024; 
        gaindb = 20*log10(pkpk / attenuated);
        break;   
    }
  vga_in = (gaindb - 12.65) / 19.7;
  dac_output_analog = (vga_in + 1.8) / 2;
  dac_output_digitized = (dac_output_analog)*(0xfff+1)/1.8;
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_output_digitized);
}

void afe_set_offset() {
 // float offset= 0.9;
//  offset = (offset)*(0xfff+1)/1.8;
  HAL_DAC_SetValue(&hdac3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 2155);
}

void DMA1_Channel1_IRQHandler(void)
{
  if(LL_DMA_IsActiveFlag_TC1(DMA1) && !LL_DMA_IsActiveFlag_HT1(DMA1)){//adc1 checks if all items in buffer were filled
    HAL_ADC_Stop_DMA(&hadc1);
  }
  HAL_DMA_IRQHandler(&hdma_adc1);
}

void DMA1_Channel2_IRQHandler(void)
{
  if(LL_DMA_IsActiveFlag_TC2(DMA1) && !LL_DMA_IsActiveFlag_HT2(DMA1)){//adc4
    HAL_ADC_Stop_DMA(&hadc4);
  }
  HAL_DMA_IRQHandler(&hdma_adc4);
}

void DMA2_Channel1_IRQHandler(void)
{
  if(LL_DMA_IsActiveFlag_TC1(DMA2) && !LL_DMA_IsActiveFlag_HT1(DMA2)){ //adc2
    HAL_ADC_Stop_DMA(&hadc2);
  }
  HAL_DMA_IRQHandler(&hdma_adc2);
}
//This is last ADC done, so here is where we stop the timer and start transmitting data
void DMA2_Channel2_IRQHandler(void)
{
  if(LL_DMA_IsActiveFlag_TC2(DMA2) && !LL_DMA_IsActiveFlag_HT2(DMA2)){ //adc5
    HAL_ADC_Stop_DMA(&hadc5);
    // Till this point, this sequence of the tranmission is done. Pause the hrtim for now, and focus on transmitting the data.
    afe_sampling_pause();
    // NOTE: Indicate the send data thread to transmit the sampled data.
    osSignalSet(send_data_task, DATA_TRANS_SIG);
  }
  HAL_DMA_IRQHandler(&hdma_adc5);
}

/** REVIEW: Some design ideas: 
 *  Need to create a global struct that stores the parameters.
 *  Need to have a function to set the datalength and sampling frequency.
 *  Before applying the new settings, need to check if it's the same as before. If it's same, don't stop the ADC for a better performance.
 *  Operation sequence:
 *  - Turn the ADC off first.
 *  - Reconfigure the ADC.
 *  - Turn the ADC on.
 */
