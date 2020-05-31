/**
 * @file tasks.h
 * @author boning@ucsb.edu
 * @brief This file includes all of the threads that may run by this project.
 * @version 0.1
 * @date 2020-05-27
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __STHREADS_H__
#define __STHREADS_H__

#include "cmsis_os.h"

#define USER_INPUT_SIG 0x01
#define DATA_TRANS_SIG 0x02

extern osThreadId send_cmd_task;
extern osThreadId send_data_task;
extern osThreadId exec_cmd_task;
extern osThreadId wait_uart_task;
extern osThreadId event_handle_task;

void tasks_initialization();
void task_send_command();
void task_send_data();
void task_exec_command();
void task_listen_uart();
void task_handle_event();

#endif