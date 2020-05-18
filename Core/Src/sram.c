#include "sram.h"
#include "string.h"

#define WRITE_ADDRESS 0x0800

uint16_t tx_buffer[256];
uint16_t rx_buffer[256];

void fill_buffer(uint16_t* buffer, uint16_t size, uint16_t data) {
  for(int i = 0; i < size; i++) {
    *(buffer + i) = data;
  }  
}

void write_memory(uint32_t address, uint16_t* buffer, uint16_t size, uint16_t stride) {
  uint16_t* cell = NULL;
  for(int i = 0; i < size; i++) {
    cell = (uint16_t*)(SRAM_BANK_ADDRESS + address + i * stride * sizeof(uint16_t));
    *cell = buffer[i];
  }
}

void read_memory(uint32_t address, uint16_t* buffer, uint16_t size, uint16_t stride) {
  uint16_t* cell = NULL;
  for(int i = 0; i < size; i++) {
    cell = (uint16_t*)(SRAM_BANK_ADDRESS + address + i * stride * sizeof(uint16_t));
    buffer[i] = *cell;
  }
}