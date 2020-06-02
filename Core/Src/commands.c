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
#include "communication.h"
#include "commands.h"
#include "cmsis_os.h"
#include "afe.h"
#include <stdlib.h>
#include <stdio.h>


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

/**
 * @brief Add a command into the command-to-send queue.
 * 
 * @param   cmd An command_t object.
 * @note    Before calling this function, need to wait for the 
 *          sem_send_slots to make sure there are avaliable spaces.
 */
void command_send_enqueue(command_t cmd) {
  uint8_t index = send_commands_buff.tail_index;
  send_commands_buff.commands[index] = cmd;
  index += 1;
  send_commands_buff.tail_index = index % CMD_BUFF_SIZE;
}

/**
 * @brief Get the earliest command in the command-to-send queue.
 * 
 * @return An command_t object.
 * @note    Before calling this function, need to wait for the 
 *          sem_commands_send to make sure there are commands needed to be send.
 */
command_t command_send_dequeue() {
  uint8_t index = send_commands_buff.head_index;
  command_t cmd = send_commands_buff.commands[index];
  index += 1;
  send_commands_buff.head_index = index % CMD_BUFF_SIZE;
  return cmd;
}

/**
 * @brief Add a command into the command-received queue.
 * 
 * @param   cmd An command_t object.
 * @note    Before calling this function, need to wait for the 
 *          sem_recv_slots to make sure there are avaliable spaces.
 */
void command_recv_enqueue(command_t cmd) {
  uint8_t index = recv_commands_buff.tail_index;
  recv_commands_buff.commands[index] = cmd;
  index += 1;
  recv_commands_buff.tail_index = index % CMD_BUFF_SIZE;
}

/**
 * @brief Add a command into the command-received queue.
 * 
 * @return  An command_t object.
 * @note    Before calling this function, need to wait for the 
 *          sem_commands_recv to make sure there are commands.
 */
command_t command_recv_dequeue() {
  uint8_t index = recv_commands_buff.head_index;
  command_t cmd = recv_commands_buff.commands[index];
  index += 1;
  recv_commands_buff.head_index = index % CMD_BUFF_SIZE;
  return cmd;
}

/**
 * @brief Validate if the input command type is a valid type.
 * 
 * @param type    A command type defined in commands.h
 * @return true   The input type is not valid.
 * @return false  The input type is valid.
 * @note  This function should be updated with the commands in commands.h.
 */
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

/**
 * @brief Executing the input command.
 * 
 * @param cmd Pointer to the command_t object.
 */
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

/**
 * @brief Contstrut a command by filling its body and it's argc.
 * This function is needed because we cannot construct the command in interrupt. 
 * Because we cannot safely allocate memory in ISR.
 * 
 * @param cmd A pointer to a cmd that has a valid type.
 */
void command_construct(command_t* cmd) {
  if(cmd == NULL)
    return;
  uint8_t type = cmd->type;
  switch (type)
  {
  case CMD_CHANGE_SEL:
  case CMD_SWIPE_DOWN:
  case CMD_SWIPE_UP:
    cmd->argc = 1;
    cmd->argv = os_malloc(1);
    *(cmd->argv) = 0xFF;
    break;
  default:
    printf("[CMD CONSTRUCT] Invalid cmd type.\r\n");
    break;
  }
}

/**
 * @brief Handle the start sampling event. If the sampling has already been started.
 * Then there is no effect.
 * 
 * @param cmd Pointer to the command_t object.
 */
static void _start_sample_handle(command_t* cmd) {
  if (afe_is_sampling_enabled())
    return;
  // Enable the sampling first to turn the global switch on.
  afe_sampling_enable();
  // Then trigger the first sampling.
  afe_sampling_trigger();
}

/**
 * @brief Handle the stop sampling event. If the sampling has already been stopped.
 * Then there is no effect.
 * 
 * @param cmd Pointer to the command_t object.
 */
static void _stop_sample_handle(command_t* cmd) {
  if (!afe_is_sampling_enabled())
    return;
  // Pause the sampling timer first.
  afe_sampling_pause();
  // Then disable the global switch.
  afe_sampling_disable();
}

/**
 * @brief Handle the battery checking event.
 * 
 * @param cmd Pointer to the command_t object.
 */
static void _check_bat_handle(command_t* cmd) {
  printf("Check voltage.\r\n");
}

/**
 * @brief Handle the voltage set event. (affects gain based on the voltage setting.)
 * 
 * @param cmd Pointer to the command_t object.
 */
static void _set_volt_handle(command_t* cmd) {
  printf("Set voltage.\r\n");
  // TODO: Need to implement this to set the gain.
}

/**
 * @brief Handle the sample parameters set event. 
 * 
 * @param cmd Pointer to the command_t object.
 */
static void _set_sample_paras_handle(command_t* cmd) {
  printf("Setting sample.\r\n");
  if (cmd == NULL) {
    printf("Invalid cmd pointer.\r\n");
    return;
  }

  if (cmd->argc != 5) {
    printf("Invalid arg size.\r\n");
    return;
  }
  uint8_t speed_option, curr_speed_option;
  uint32_t sample_length, curr_sample_length;
  // Parse the speed option and length from the new command.
  speed_option = cmd->argv[0];
  sample_length = (cmd->argv[1] << 24) | (cmd->argv[2] << 16) | (cmd->argv[3] << 8) | (cmd->argv[4]); 
  // Fetach the current setting.
  afe_get_current_sampling_paras(&curr_speed_option, &curr_sample_length);
  // Check if the parsed sampling parameters are the same as the current one.
  // If it's not, then reset the sampling parameters.
  if (speed_option != curr_speed_option || sample_length != curr_sample_length) {
    // Turn off the ADC sampling first.
    if (!afe_is_sampling_paused())
      afe_sampling_pause();
    // Set the sampling parameters
    afe_set_sampling_paras(speed_option, sample_length);
    // Reenable the ADC sampling if it's running.
    if (afe_is_sampling_enabled())
      afe_sampling_trigger();
  }
}