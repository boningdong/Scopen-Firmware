/**
 * @file commands.c
 * @author boning@ucsb.edu
 * @brief This file contains the function that relates to parsing and processing the commands.
 * @version 0.1
 * @date 2020-05-17
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "main.h"
#include <stdlib.h>
#include "communication.h"
#include "commands.h"
#include "cmsis_os.h"


// Static function declarations
static void _start_sample_handle(command_t* cmd);
static void _stop_sample_handle(command_t* cmd);
static void _check_bat_handle(command_t* cmd);
static void _set_volt_handle(command_t* cmd);
static void _set_sample_paras_handle(command_t* cmd);

command_t latest_cmd = {0};

/**
 * @brief sem_commands_ready indicates whether the command handling thread needs to transfer user inputs.
 * 
 */
osSemaphoreId sem_send_slots;
osSemaphoreId sem_recv_slots;
osSemaphoreId sem_commands_send;
osSemaphoreId sem_commands_recv;

command_buff_t send_commands_buff = {0};
command_buff_t recv_commands_buff = {0};

/**
 * @brief Initialize the command processor, including initializing the 
 * commands buffer and corresponding semaphores.
 * 
 */
void command_processor_init() {
  // initialize the semaphores
  osSemaphoreDef(samples_ready);
  osSemaphoreDef(send_slots);
  osSemaphoreDef(recv_slots);
  osSemaphoreDef(send_cmds);
  osSemaphoreDef(recv_cmds);
  sem_send_slots = osSemaphoreCreate(osSemaphore(send_slots), CMD_BUFF_SIZE);
  sem_recv_slots = osSemaphoreCreate(osSemaphore(recv_slots), CMD_BUFF_SIZE);
  sem_commands_send = osSemaphoreCreate(osSemaphore(send_cmds), 1);
  sem_commands_recv = osSemaphoreCreate(osSemaphore(recv_cmds), 1);
  // initialize the ring buffers.
  send_commands_buff.head_index = 0;
  send_commands_buff.tail_index = 0;
  recv_commands_buff.head_index = 0;
  recv_commands_buff.tail_index = 0;
  
}

void command_send_enqueue(command_t cmd) {
  osSemaphoreWait(sem_send_slots, osWaitForever);
  uint8_t index = send_commands_buff.tail_index;
  send_commands_buff.commands[index] = cmd;
  index += 1;
  send_commands_buff.tail_index = index % CMD_BUFF_SIZE;
}

command_t command_send_dequeue() {
  uint8_t index = send_commands_buff.head_index;
  command_t cmd = send_commands_buff.commands[index];
  index += 1;
  send_commands_buff.head_index = index % CMD_BUFF_SIZE;
  return cmd;
}

void command_recv_enqueue(command_t cmd) {
  uint8_t index = recv_commands_buff.tail_index;
  recv_commands_buff.commands[index] = cmd;
  index += 1;
  recv_commands_buff.tail_index = index % CMD_BUFF_SIZE;
}

command_t command_recv_dequeue() {
  uint8_t index = recv_commands_buff.head_index;
  command_t cmd = recv_commands_buff.commands[index];
  index += 1;
  recv_commands_buff.head_index = index % CMD_BUFF_SIZE;
  return cmd;
}
 
bool command_validate(uint8_t type) {
  switch(type) {
  case CMD_START_SAMPLE:
  case CMD_STOP_SAMPLE:
  case CMD_CHECK_BAT:
  case CMD_SET_VOLTAGE:
  case CMD_SET_SAMPLE_PARAS:
    return true;
  default:
    return false;
  }
}

void command_execute(command_t* cmd) {
  if (cmd == NULL) {
    printf("The passed in cmd pointer is null.\r\n");
    return;
  }
  switch(cmd->type) {
    case CMD_START_SAMPLE:
      _start_sample_handle(cmd);
      break;
    case CMD_STOP_SAMPLE:
      _stop_sample_handle(cmd);
      break;
    case CMD_CHECK_BAT:
      _check_bat_handle(cmd);
      break;
    case CMD_SET_VOLTAGE:
      _set_volt_handle(cmd);
      break;
    case CMD_SET_SAMPLE_PARAS:
      _set_sample_paras_handle(cmd);
      break;
    default:
      printf("Invalid command to execute.\r\n");
  }
}

static void _start_sample_handle(command_t* cmd) {
  printf("Start sampling.\r\n");
}

static void _stop_sample_handle(command_t* cmd) {
  printf("Stop sampling.\r\n");
}

static void _check_bat_handle(command_t* cmd) {
  printf("Check voltage.\r\n");
}

static void _set_volt_handle(command_t* cmd) {
  printf("Set voltage.\r\n");
}

static void _set_sample_paras_handle(command_t* cmd) {
  printf("Set sample paras.\r\n");
}