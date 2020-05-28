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

/**
 * @brief sem_samples_ready indicates whether the controller need to transfer samples to the upper machine.
 * 
 */
osSemaphoreId sem_samples_ready;
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
  osSemaphoreDef(send_slots);
  osSemaphoreDef(recv_slots);
  osSemaphoreDef(send_cmds);
  osSemaphoreDef(recv_cmds);
  sem_send_slots = osSemaphoreCreate(osSemaphore(send_slots), CMD_BUFF_SIZE);
  sem_recv_slots = osSemaphoreCreate(osSemaphore(recv_slots), CMD_BUFF_SIZE);
  sem_commands_send = osSemaphoreCreate(osSemaphore(send_cmds), 0);
  sem_commands_recv = osSemaphoreCreate(osSemaphore(recv_cmds), 0);
  // initialize the ring buffers.
  
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


void command_send_thread() {
  for(;;) {
    osSemaphoreWait(sem_commands_send, osWaitForever);
    command_t cmd = command_send_dequeue();
    osSemaphoreRelease(sem_send_slots);
    communication_transfer_message(cmd.type, cmd.argv, cmd.argc);
    // Free the dynamic memory allocated for the cmd.
    if (cmd.argv != NULL && cmd.argc != 0)
      free(cmd.argv);
  }
}

void command_recv_thread() {
  for(;;) {
    osSemaphoreWait(sem_commands_recv, osWaitForever);
  }
}