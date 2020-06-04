#include <WiFi.h>
#include <Arduino.h>
#include <SPI.h>

#define DATSIZE 4
#define DATA_LENGTH 5
#define BUFF_SIZE 4096
#define SPI_SPEED 5000000


//This tests the SPI on ESP32
//This version tests with the interupt pin from stm32 -> esp32 on pin 15
//You can also test by disabling the attachInterrupt
//we have to swap MISO and MOSI pins, since by default in HSPI are swapped
const int MISOPin = 13;
const int MOSIPin =  12;
const int SCKPin = 14;
const int slaveSelectPin = 15;

int comm_count = 1;

uint8_t data_buffer[BUFF_SIZE];

void spi_read_size(uint16_t length);


const int interruptPin = 23;
int count = 0;
uint8_t dataS[14]; 
SPIClass spi(HSPI);
uint8_t readFlag = 0;

  
void IRAM_ATTR flagReadADCData(void){
  Serial.println(comm_count);
  readFlag = 1;
}
  
void setup()
{
  Serial.begin(115200);
  Serial.println();
  
  pinMode(slaveSelectPin, OUTPUT);
  digitalWrite(slaveSelectPin, HIGH);
  attachInterrupt(interruptPin, flagReadADCData, RISING); //comment this out if you want to test without stm32->esp32 interrupt
  spi.begin(SCKPin,MISOPin,MOSIPin,slaveSelectPin);

}

void loop()
{
  //writeToMCU(2);

  // if(readFlag){ //comment this block out if you want ESP32 to poll data instead of waiting for interrupt
  //   readFlag = 0;
  //   readIncoming();
  //   Serial.println("Received:");
  //   for(int i = 0; i < READ_SIZE; i++){
  //     Serial.println(data_buffer[i]);
  //     data_buffer[i] = 0;
  //   }   
  // }
  uint32_t left_length = DATA_LENGTH;
  if(readFlag) {
    readFlag = 0;
    while(left_length && !readFlag) {
      uint16_t read_length = left_length > BUFF_SIZE ? BUFF_SIZE : left_length;
      spi_read_size(read_length);
      for(int i = 0; i < read_length; i++) {
        Serial.print(data_buffer[i], HEX);
        Serial.print(' ');
        data_buffer[i] = 0;
      }
      left_length -= read_length;
    }
    Serial.print('\n');
    writeToMCU('A');
    comm_count++;
  }

  /*
  count++; //uncoment this if u want esp32 to poll incoming data
  if(count == 1000){
    count = 0;
    readIncoming();
    Serial.println("Received:");
    for(int i = 0; i<DATSIZE; i++){
      Serial.println(data_buffer[i]);
      data_buffer[i] = 0;
    }   
  }*/
}



void readIncoming(){
  int del = 10;
  spi.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
  digitalWrite(slaveSelectPin, LOW);
  delay(del);
  spi.transferBytes(NULL,data_buffer, BUFF_SIZE);
  digitalWrite(slaveSelectPin, HIGH);
  delay(del);
  spi.endTransaction(); 
}

void spi_read_size(uint16_t length) {
  int del = 10;
  spi.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
  digitalWrite(slaveSelectPin, LOW);
  delay(del);
  spi.transferBytes(NULL,data_buffer, length);
  digitalWrite(slaveSelectPin, HIGH);
  delay(del);
  spi.endTransaction(); 
}


void writeToMCU(uint8_t cmd){
  int del = 10;
  spi.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
  digitalWrite(slaveSelectPin, LOW);
  delay(del);
  spi.write(cmd);
  delay(del);
  spi.endTransaction();
  digitalWrite(slaveSelectPin, HIGH);
}
