/**
 * @file sthreads.c
 * @author boning@ucsb.edu
 * @brief This file includes all of the threads that may run by this project.
 * @version 0.1
 * @date 2020-05-27
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <stdlib.h>
#include "sthreads.h"
#include "cmsis_os.h"
#include "main.h"
#include "commands.h"
#include "communication.h"
#include <stdio.h>
#include "sram.h"
#include "afe.h"
/**
 * @defgroup Upstream communication handlers.
 * 
 */

osThreadId send_cmd_task;
osThreadId send_data_task;
osThreadId exec_cmd_task;
osThreadId wait_uart_task;
osThreadId event_handle_task;

void tasks_initialization() {
  osThreadDef(SendCmd, task_send_command, osPriorityNormal, 0, 64);
  osThreadDef(SendData, task_send_data, osPriorityHigh, 0, 64);
  osThreadDef(ExecCmd, task_exec_command, osPriorityNormal, 0, 64);
  osThreadDef(WaitUart, task_listen_uart, osPriorityNormal, 0, 64);
  osThreadDef(EventHandle, task_handle_event, osPriorityRealtime, 0, 64);
  send_cmd_task = osThreadCreate(osThread(SendCmd), NULL);
  send_data_task = osThreadCreate(osThread(SendData), NULL);
  exec_cmd_task = osThreadCreate(osThread(ExecCmd), NULL);
  wait_uart_task = osThreadCreate(osThread(WaitUart), NULL);
  event_handle_task = osThreadCreate(osThread(EventHandle), NULL);
}

/**
 * @brief This thread will be unblocked as long as the commands-to-send buffer is not empty.
 * It will tranfer the command from the pen to the upper machine.
 */
void task_send_command() {
  printf("Send Command thread has been initialized.\r\n");
  // NOTE: P the semaphore to make the sempahore function as a binary semaphore.
  osSemaphoreWait(sem_commands_send, osWaitForever);
  for(;;) {
    // Send command
    osSemaphoreWait(sem_commands_send, osWaitForever);
    printf("Sending pen cmd to PC.\r\n");
    command_t cmd = command_send_dequeue();
    osSemaphoreRelease(sem_send_slots);
    communication_transfer_message(cmd.type, cmd.argv, cmd.argc);
    // Free the dynamic memory allocated for the cmd.
    if (cmd.argv != NULL && cmd.argc != 0)
      free(cmd.argv);
    
  }
}

/**
 * @brief This thread will be unblocked as long as the ADC is done with one round of sampling.
 * It will transfer the ADC results to the upper machine. 
 */
void task_send_data() {
  printf("Send Data thread has been initialized.\r\n");
  // NOTE: P the semaphore to make the sempahore function as a binary semaphore.
  osSemaphoreWait(sem_transfer_done, osWaitForever);
  for(;;) {
    // NOTE: Pause off ADC sampling first.
    // The ADC sampling is paused inside the DMA function.
    // And the DMA function will send the following signal.
    osSignalWait(DATA_TRANS_SIG, osWaitForever);
    printf("Sending data to PC.\r\n");
    // NOTE: Because the data is saved in the external SRAM here. So we use the SRAM memory address.
    // NOTE: Even though the SRAM is 16bit width memory, we use uint8_t* here to follow the function signature.
    //       SPI DMA will handle the conversion from the 16 bit width data to the SPI register directly.
    uint8_t* result_buffer = (uint8_t*) SRAM_BANK_ADDRESS;
    communication_transfer_message(CMD_DATA, result_buffer, last_conv_length);
    // NOTE: check if the sampling global switch is enabled. If it is enabled run a new sequence of sampling again.
    if(afe_is_sampling_enabled())
      afe_sampling_trigger();
  }
}


/**
 * @defgroup Downstream communication handlers.
 * 
 */

/**
 * @brief This thread will be unblocked as long as the received commands buffer is not empty.
 * It will execute the commands based on the FIFO order.
 * 
 */
void task_exec_command() {
  printf("Exec Command thread has been initialized.\r\n");
  osSemaphoreWait(sem_commands_recv, osWaitForever);
  for(;;) {
    osSemaphoreWait(sem_commands_recv, osWaitForever);
    printf("Handling received cmd.\r\n");
    command_t cmd = command_recv_dequeue();
    osSemaphoreRelease(sem_recv_slots);
    command_execute(&cmd);
    if(cmd.argv != NULL && cmd.argc != 0) {
      free(cmd.argv);
    }
    printf("Done with a cmd.\r\n");
  }
}

/**
 * @brief This thread is a listener for the uart input. It will keep reading the uart and append the
 * received commands from PC to the received commands queue.
 * @note This may be implemented using dynamic thread.
 */
void task_listen_uart() {
  printf("Listen UART thread has been initialized.\r\n");
  ErrorStatus status = 0;
  uint8_t header_buffer[HEADER_SIZE] = {0};
  uint8_t type = 0xFF;
  uint8_t ack = 'A';
  uint8_t *body_buffer = NULL;
  uint32_t length = 0;
  for(;;) {
    status = communication_uart_receive(header_buffer, HEADER_SIZE, UART_WAIT_HEADER_TIMEOUT);
    if (status != SUCCESS)
      continue;
    printf("Uart received header\r\n");

    // parse header
    length = 0;
    for (int i = 0; i < HEADER_SIZE_FIELD; i++) {
      uint8_t offset = HEADER_SIZE_FIELD - i - 1;
      length |= header_buffer[i] << (sizeof(uint8_t) * offset);
    }

    type = header_buffer[HEADER_SIZE - 1];
    // Check the validity of the type. If it's not validate, go back to wait for the header again.
    if (!command_validate(type)) {
      printf("Uart received invalid type.[Header]\r\n");
      continue;
    }
    
    // Check the type is valid. Reply with 'A'.
    status = communication_uart_send(&ack, 1, UART_SEND_ACK_TIMEOUT);
    if (status != SUCCESS) {
      printf("Uart sent ack failed.\r\n");
      continue;
    }
    
    // Wait for the body
    
    // REVIEW: Here the malloc may be not thread-safe. Change to thread-safe one if it's necessary.
    if (length > 0) {
      body_buffer = (uint8_t*)malloc(sizeof(uint8_t) * length);
      // NOTE: Here the cast should be fine. Because all of the commands will not have long body.
      status = communication_uart_receive(body_buffer, (uint16_t) length, UART_WAIT_BODY_TIMEOUT);
      if (status != SUCCESS) {
        printf("Uart receive body failed.\r\n");
        free(body_buffer);
        continue;
      }
    } else {
      printf("Invalid length received: %lu\r\n", length);
      continue;
    }
    
    status = communication_uart_send(&ack, 1, UART_SEND_ACK_TIMEOUT);
    if (status != SUCCESS) {
      printf("Uart sent ack failed.[Body]\r\n");
      free(body_buffer);
      continue;
    }
    
    command_t cmd = {0};
    cmd.type = type;
    cmd.argc = length;
    cmd.argv = body_buffer;
    osSemaphoreWait(sem_recv_slots, osWaitForever);
    command_recv_enqueue(cmd);
    osSemaphoreRelease(sem_commands_recv);
  } 
}

/**
 * @defgroup Dynamic threads. 
 * 
 */

/**
 * @brief A dynamic task will be allocated by the touch interrupt.
 * 
 * @param args A void pointer that should be able to cast to command_t*.
 */
void task_handle_event() {
  printf("Touch event handling thread.\r\n");
  for(;;) {
    osSignalWait(USER_INPUT_SIG, osWaitForever);
    // Check whether there are enough slots.
    osSemaphoreWait(sem_send_slots, osWaitForever);
    command_send_enqueue(latest_cmd);
    osSemaphoreRelease(sem_commands_send);
  }
} 