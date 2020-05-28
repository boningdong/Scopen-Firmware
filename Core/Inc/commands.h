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

  /**
   * Command type constants - Pen to Software
   */
 
#define CMD_DATA              0x00
#define CMD_REPORT_BAT        0x01
#define CMD_SWIPE_UP          0x11
#define CMD_SWIPE_DOWN        0x12
#define CMD_CHANGE_SEL        0x13

  /**
   * Command type constants - Software to Pen
   */
#define CMD_START_SAMPLE      0x21
#define CMD_STOP_SAMPLE       0x22
#define CMD_CHECK_BAT         0x23
#define CMD_SET_VOLTAGE       0x41
#define CMD_SET_SAMPLE_PARAS  0x42

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

void command_processor_init();
void command_send_thread();
void command_recv_thread();