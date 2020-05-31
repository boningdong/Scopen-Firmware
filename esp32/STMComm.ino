bool verifyCMDFromSTM(const uint8_t &dataType){
  return dataType == CMD_DATA || dataType == CMD_REPORT_BAT
         || dataType == CMD_SWIPE_UP || dataType == CMD_SWIPE_DOWN
         || dataType == CMD_CHANGE_SEL;
}

bool sendHeaderSTM(const uint32_t &dataSize, const uint8_t &dataType){
  uint8_t header[HEADER_SIZE];
  constructHeader(header,dataSize,dataType);
  Serial2.write(header,HEADER_SIZE);
  Serial2.flush();
  return waitForACKSTM(10);
}

bool writeMessageSTM(const uint8_t* msg, const uint32_t &dataLength){
  Serial.print("Sending message to STM: ");
  Serial.println(Serial2.write(msg, dataLength));
  Serial2.flush();
  return waitForACKSTM(10);
}

void sendACKSTM(){
  int del = 10;
  spi.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
  digitalWrite(SS_PIN,LOW);
  delay(del);
  spi.write('A');
  delay(del);
  digitalWrite(SS_PIN,HIGH);
  spi.endTransaction();
  Serial.println("Sent ACK to STM");
}

bool waitForACKSTM(int timeout){
  unsigned long sec = millis();
  while(!(Serial2.available()>0)&& !((millis()-sec)>timeout)){} //need timeout
  if(Serial2.read() == (int) 'A'){
    Serial.println("Recieved ACK UART");
    return true;
  }
  else{
    Serial.println("Wrong ACK recieved from UART");
  }
  
  return false;
}

void readHeaderSTM(uint32_t &spiDataSize, uint8_t &spiDataType){
    uint8_t header[HEADER_SIZE];

    int del = 1;
    spi.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
    digitalWrite(SS_PIN,LOW);
    delay(del);
    spi.transferBytes(NULL,header,HEADER_SIZE);
    digitalWrite(SS_PIN,HIGH);
    delay(del);
    spi.endTransaction();
    for(int i = 0; i<5;i++){
       Serial.print(header[i]);Serial.print(" ");
    }
     Serial.println("");
    parseBigEndian(header, spiDataSize);
    spiDataType = header[HEADER_SIZE-1];
    Serial.print("Data size: "); Serial.println(spiDataSize);
    Serial.print("Data type: "); Serial.println(spiDataType);
}

void readMessageSTM(uint8_t* msg, const uint32_t &spiDataLength){
  int del = 1;
  spi.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
  digitalWrite(SS_PIN,LOW);
  delay(del);
  spi.transfer(msg,spiDataLength);
  digitalWrite(SS_PIN,HIGH);
  delay(del);
  spi.endTransaction();
}
