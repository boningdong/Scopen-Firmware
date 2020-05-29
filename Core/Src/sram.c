#include "sram.h"
#include "string.h"

#define WRITE_ADDRESS 0x0800

uint16_t tx_buffer[256];
uint16_t rx_buffer[256];

void sram_fill_buffer(uint16_t* buffer, uint16_t size, uint16_t data) {
  for(int i = 0; i < size; i++) {
    *(buffer + i) = data;
  }  
}

void sram_rand_write(uint32_t address, uint16_t* buffer, uint16_t size, uint16_t stride) {
  uint16_t* cell = NULL;
  for(int i = 0; i < size; i++) {
    cell = (uint16_t*)(SRAM_BANK_ADDRESS + address + i * stride * sizeof(uint16_t));
    *cell = buffer[i];
  }
}

void sram_rand_read(uint32_t address, uint16_t* buffer, uint16_t size, uint16_t stride) {
  uint16_t* cell = NULL;
  for(int i = 0; i < size; i++) {
    cell = (uint16_t*)(SRAM_BANK_ADDRESS + address + i * stride * sizeof(uint16_t));
    buffer[i] = *cell;
  }
}

void sram_rand_write_embedded(uint32_t address, uint8_t* buffer, uint8_t size) {
  uint8_t* cell = (uint8_t*) CCMSRAM_BASE + address;
  for (int i = 0; i < size; i++) {
    cell[i] = buffer[i];
  }
}

void sram_rand_read_embedded(uint32_t address, uint8_t* buffer, uint8_t size) {
  uint8_t* cell = (uint8_t*) CCMSRAM_BASE + address;
  for (int i = 0; i < size; i++) {
    buffer[i] = cell[i];
  }
}