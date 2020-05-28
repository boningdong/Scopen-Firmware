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

void tasks_initialization();
void task_send_command();
void task_send_data();
void task_exec_command();
void task_listen_uart();
void task_handle_event(void* args);

#endif