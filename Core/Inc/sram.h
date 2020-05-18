#ifndef __SRAM_H__
#define __SRAM_H__

#include "main.h"

#define SRAM_BANK_ADDRESS 0x60000000

extern uint16_t tx_buffer[256];
extern uint16_t rx_buffer[256];

void fill_buffer(uint16_t* buffer, uint16_t size, uint16_t data);
void write_memory(uint32_t address, uint16_t* buffer, uint16_t size, uint16_t stride);
void read_memory(uint32_t address, uint16_t* buffer, uint16_t size, uint16_t stride);

#endif