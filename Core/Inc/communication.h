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


#define MAX_SPI_BUFFER_SIZE 4096
#define HEADER_SIZE 5
#define HEADER_SIZE_FIELD 4
#define HEADER_TYPE_FIELD 1
#define WAIT_ACK_TIMEOUT 2000

typedef struct {
  uint8_t header_sent;
  uint8_t header_ack;
  uint8_t body_sent;
  uint8_t body_ack;
  uint16_t started_ticks;
  uint16_t size;
  uint8_t* buffer;
} transfer_status_t;

void communication_transfer_message(uint8_t type, uint8_t* buffer, uint32_t length);


