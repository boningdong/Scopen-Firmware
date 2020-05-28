/**
 * @file commands.h
 * @author boning@ucsb.edu
 * @brief This file contains the function that relates to parsing and processing the commands.
 * @version 0.1
 * @date 2020-05-17
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __COMMANDS_H__
#define __COMMANDS_H__

#include "main.h"
#include "cmsis_os.h"

  /**
   * Command type constants - Pen to Software
   */
 
#define CMD_DATA              0x00      
#define CMD_REPORT_BAT        0x01        // Format [length:4, type:1][battery percent: 1]
#define CMD_SWIPE_UP          0x11        // Format [length:4, type:1][0xFF]
#define CMD_SWIPE_DOWN        0x12        // Format [length:4, type:1][0xFF]
#define CMD_CHANGE_SEL        0x13        // Format [length:4, type:1][0xFF]

  /**
   * Command type constants - Software to Pen
   */
#define CMD_START_SAMPLE      0x21        // Format [length:4, type:1][0xFF]
#define CMD_STOP_SAMPLE       0x22        // Format [length:4, type:1][0xFF]
#define CMD_CHECK_BAT         0x23        // Format [length:4, type:1][0xFF]
#define CMD_SET_VOLTAGE       0x41        // Format [length:4, type:1][Voltage Div Index: 1]
#define CMD_SET_SAMPLE_PARAS  0x42        // Format [length:4, type:1][Sampling Speed Index: 1][Buffer length: 4]

#define CMD_BUFF_SIZE    16

typedef struct {
  uint8_t type;
  uint8_t *argv;
  uint8_t argc;
} command_t;


typedef struct {
  command_t commands[CMD_BUFF_SIZE];
  uint8_t head_index;                 // The index of the first occupied slot.
  uint8_t tail_index;                 // The index of the first available slot
} command_buff_t;


// Extern variables
extern osSemaphoreId sem_samples_ready;
extern osSemaphoreId sem_send_slots;
extern osSemaphoreId sem_recv_slots;
extern osSemaphoreId sem_commands_send;
extern osSemaphoreId sem_commands_recv;
extern command_buff_t send_commands_buff;
extern command_buff_t recv_commands_buff;

// Function delcarations
void command_processor_init();
void command_send_enqueue(command_t cmd);
void command_recv_enqueue(command_t cmd);
command_t command_send_dequeue();
command_t command_recv_dequeue();
bool command_validate(uint8_t type);
void command_execute();

#endif