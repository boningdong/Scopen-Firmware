/**
 * @file communication.h
 * @author boning@ucsb.edu
 * @brief This file relates to the communication between the STM32 and ESP32.
 * @version 0.1
 * @date 2020-05-17
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#define MAX_SPI_BUFFER_SIZE 4096
#define HEADER_SIZE 5
#define HEADER_SIZE_FIELD 4
#define HEADER_TYPE_FIELD 1
#define SPI_WAIT_ACK_TIMEOUT 2000
#define UART_WAIT_HEADER_TIMEOUT 1000
#define UART_WAIT_BODY_TIMEOUT 1000
#define UART_SEND_ACK_TIMEOUT 1000

typedef struct {
  uint8_t header_sent;
  uint8_t header_ack;
  uint8_t body_sent;
  uint8_t body_ack;
  uint16_t started_ticks;
  uint16_t size;
  uint8_t* buffer;
} transfer_status_t;

void communication_initialization();
void communication_transmit(uint8_t* buffer, uint16_t count);
void communication_transfer_message(uint8_t type, uint8_t* buffer, uint32_t length);
ErrorStatus communication_uart_send(uint8_t* buffer, uint16_t size, uint32_t timeout);
ErrorStatus communication_uart_receive(uint8_t* buffer, uint16_t size, uint32_t timeout);

#endif