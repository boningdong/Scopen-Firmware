#ifndef __SRAM_H__
#define __SRAM_H__

#include "main.h"

#define SRAM_BANK_ADDRESS 0x60000000

extern uint16_t tx_buffer[256];
extern uint16_t rx_buffer[256];

void sram_fill_buffer(uint16_t* buffer, uint16_t size, uint16_t data);
void sram_rand_write(uint32_t address, uint16_t* buffer, uint16_t size, uint16_t stride);
void sram_rand_read(uint32_t address, uint16_t* buffer, uint16_t size, uint16_t stride);

void sram_rand_write_embedded(uint32_t address, uint8_t* buffer, uint8_t size);
void sram_rand_read_embedded(uint32_t address, uint8_t* buffer, uint8_t size);

#endif