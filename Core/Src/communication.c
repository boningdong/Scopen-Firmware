/**
 * @file communication.c
 * @author boning@ucsb.edu
 * @brief This file relates to the communication between the STM32 and ESP32.
 * @version 0.1
 * @date 2020-05-17
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "main.h"
#include "communication.h"
#include "cmsis_os.h"
#include "commands.h"
#include <stdio.h>

static void _init_spi();
static void _init_uart();
static void _make_header(uint8_t* header, uint8_t type, uint32_t length);
static void _init_transfer_semaphore();

UART_HandleTypeDef huart1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;


osSemaphoreId sem_transfer_occupied;

/**
 * @brief Initialize the communication interface including spi and uart. And necessary semaphores.
 * 
 */
void communication_initialization() {
  _init_spi();
  _init_transfer_semaphore();
  _init_uart();
}

// If the following functions need to be implemented in the interrupts, 
// you can write comments inside these functions to tell me where to find the corresponding code.
// Try to define the communication related functions and interrupts in this file.

void communication_transmit(uint8_t* buffer, uint16_t count) {
  HAL_SPI_Transmit_DMA(&hspi3, buffer, count);
}

void communication_receive(uint8_t* buffer, uint16_t count) {
  HAL_SPI_RECEIVE_DMA(&hspi3,buffer,count);
}

ErrorStatus communication_receive_block(uint8_t* buffer, uint16_t count, uint32_t timeout) {
  return HAL_SPI_Receive(&hspi3, buffer, count, timeout);
}

/**
 * @brief This function will transmit the formated packet to esp32 so that esp32 can forward the packet to tcp.
 *        
 * @param type    Command type. Defined in the commands.h
 * @param buffer  Payload of the message. Can be address from SRAM or from any buffer.
 * @param length  Payload transfer length.
 */
void communication_transfer_message(uint8_t type, uint8_t* buffer, uint32_t length) {
  // REVIEW: May need to change this function to return a boolean status.
  ErrorStatus status;
  uint8_t ack = 0;
  // Make the header of the communication.
  uint8_t header[HEADER_SIZE];
  _make_header(header, type, length);
  
  osSemaphoreWait(sem_transfer_occupied, osWaitForever);
  // Transfer the header of the data.
  communication_transmit(header, HEADER_SIZE);
  // Wait for the acknowledgement.
  
  status = communication_receive_block(&ack, 1, SPI_WAIT_ACK_TIMEOUT);
  if (status != SUCCESS || status != 'A') {
    // Release the shared resources and stop.
    printf("Didn't received valid header ACK\r\n");
    osSemaphoreRelease(sem_transfer_occupied);
    return;
  }
  
  // Transfer the data.
  while (length) {
    // Transfer the body
    uint16_t transfer_size = (length <= MAX_SPI_BUFFER_SIZE) ? length : MAX_SPI_BUFFER_SIZE;
    communication_transmit(buffer, transfer_size);
    // Wait for the acknowledgement;
    status = communication_receive_block(&ack, 1, SPI_WAIT_ACK_TIMEOUT);
    if (status != SUCCESS || status != 'A') {
      // Release the shared resources and stop.
      printf("Didn't received valid body ACK\r\n");
      osSemaphoreRelease(sem_transfer_occupied);
      return;
    }
    length -= transfer_size;
    buffer += transfer_size;
  }
  
  // Release the shared resources and stop.
  printf("Transfer succeed.\r\n");
  osSemaphoreRelease(sem_transfer_occupied);
}


/**
 * @brief Wrapper function for the uart communication on this platform.
 * 
 * @param buffer  A pointer to the buffer where the data will be saved.
 * @param size    The size to receive in bytes.
 * @param timeout Timeout.
 */
ErrorStatus communication_uart_receive(uint8_t* buffer, uint16_t size, uint32_t timeout) {
  return HAL_UART_Receive(&huart1, buffer, size, timeout);
}

/**
 * @brief Wrapper function for the uart communication on this platform.
 * 
 * @param buffer  A pointer to the buffer where the data will be read from.
 * @param size    The size to send in bytes.
 * @param timeout Timeout.
 */
ErrorStatus communication_uart_send(uint8_t* buffer, uint16_t size, uint32_t timeout) {
  return HAL_UART_Transmit(&huart1, buffer, size, timeout);
}


/**
 * @defgroup Static methods
 * 
 */

/**
 * @brief Fill the passed-in header for based on the transfering data.
 * 
 * @param header  The header array that will be filled with the length and type info.
 * @param type    Command type. Defined in the commands.h
 * @param length  The length information of the payload.
 */
static void _make_header(uint8_t* header, uint8_t type, uint32_t length) {
  header[HEADER_SIZE-1] = type;
  for (uint8_t i = 0; i < HEADER_SIZE_FIELD; i++) {
    uint8_t shift_bytes = HEADER_SIZE_FIELD - i - 1;
    header[i] = (length >> (sizeof(uint8_t) * shift_bytes)) & 0xFF;
  }
}
/**
 * @brief Initialize the tranfer occupied semaphore so that it can accept one token.
 * 
 */
static void _init_transfer_semaphore() {
  osSemaphoreDef(SemSpiBusy);
  sem_transfer_occupied = osSemaphoreCreate(osSemaphore(SemSpiBusy), 1);
  printf("SPI Tranfer Semaphore is initialized.\r\n");
}

/**
 * @brief Initialize the spi interface on the stm32. It will be used for the upstream communication.
 * Mainly for uploading the data and the pen command to esp32.
 * 
 */
static void _init_spi() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_SPI3_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /**SPI3 GPIO Configuration    
  PA15     ------> SPI3_NSS
  PC10     ------> SPI3_SCK
  PC11     ------> SPI3_MISO
  PC12     ------> SPI3_MOSI 

  PB13     ------> ESP32 Pin 23 interrupt
  */

  //This is the interrupt pin for the ESP32
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);     

  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
  /* SPI3 DMA Init */
  /* SPI3_RX Init */
  
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

  hdma_spi3_rx.Instance = DMA1_Channel3;
  hdma_spi3_rx.Init.Request = DMA_REQUEST_SPI3_RX;
  hdma_spi3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_spi3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_spi3_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_spi3_rx.Init.Mode = DMA_NORMAL;
  hdma_spi3_rx.Init.Priority = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&hdma_spi3_rx) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(&hspi3,hdmarx,hdma_spi3_rx);

  /* SPI3_TX Init */
  hdma_spi3_tx.Instance = DMA1_Channel4;
  hdma_spi3_tx.Init.Request = DMA_REQUEST_SPI3_TX;
  hdma_spi3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_spi3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_spi3_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_spi3_tx.Init.Mode = DMA_NORMAL;
  hdma_spi3_tx.Init.Priority = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&hdma_spi3_tx) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(&hspi3,hdmatx,hdma_spi3_tx);

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
      Error_Handler();
  }
}


/**
 * @brief Initialize the uart interface on the stm32. It will be used for the downstream communication.
 * Mainly for receiving the commands from the esp32.
 * 
 */
static void _init_uart() {

  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  /**USART1 GPIO Configuration    
  PC4     ------> USART1_TX
  PC5     ------> USART1_RX 
  */
 GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
 * @defgroup Interrupt handlers.
 * 
 */

/**
 * @brief DMA1 Interrupt handler. It will be called after data received.
 * 
 */
void DMA1_Channel3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi3_rx);
  HAL_SPI_DMAStop(&hspi3);
}

/**
 * @brief DMA1 Interrupt handler. It will be called after done sending.
 * 
 */
void DMA1_Channel4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
  //Triggers the ESP32 Interrupt
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
  HAL_SPI_DMAStop(&hspi3);
}
