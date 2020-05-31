bool verify_cmd_from_stm(const uint8_t &data_type){
  return data_type == CMD_DATA || data_type == CMD_REPORT_BAT
         || data_type == CMD_SWIPE_UP || data_type == CMD_SWIPE_DOWN
         || data_type == CMD_CHANGE_SEL;
}

bool send_header_stm(const uint32_t &data_size, const uint8_t &data_type){
  uint8_t header[HEADER_SIZE];
  constructHeader(header,data_size,data_type);
  Serial2.write(header,HEADER_SIZE);
  Serial2.flush();
  return wait_for_ack_stm(10);
}

bool write_message_stm(const uint8_t* msg, const uint32_t &data_length){
  Serial.print("Sending message to STM: ");
  Serial.println(Serial2.write(msg, data_length));
  Serial2.flush();
  return wait_for_ack_stm(10);
}

void send_ack_stm(){
  int del = 1;
  spi.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
  digitalWrite(SS_PIN,LOW);
  delay(del);
  spi.write('A');
  delay(del);
  digitalWrite(SS_PIN,HIGH);
  spi.endTransaction();
  Serial.println("Sent ACK to STM");
}

bool wait_for_ack_stm(int timeout){
  unsigned long sec = millis(); unsigned long current_time = millis();
  while(!(Serial2.available()>0)&& !((current_time - sec)>timeout)){
    current_time = millis();
  }
  if(current_time-sec>timeout){
    Serial.println("UART ACK timed out");
    return false;
  }
  else if(Serial2.read() == (int) 'A'){
    Serial.println("Recieved ACK UART");
    return true;
  }
  else{
    Serial.println("Wrong ACK recieved from UART");
    return false;
  }
  
}

void read_header_stm(uint32_t &spi_data_size, uint8_t &spi_data_type){
    uint8_t header[HEADER_SIZE];
    int del = 1;
    spi.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
    digitalWrite(SS_PIN,LOW);
    delay(del);
    spi.transferBytes(NULL,header,HEADER_SIZE);
    digitalWrite(SS_PIN,HIGH);
    delay(del);
    spi.endTransaction();
    Serial.println(" ");
    Serial.print("SPI Header: ");
    for(int i = 0; i<5;i++){
       Serial.print(header[i]);Serial.print(" ");
    }
     Serial.println("");
    parseBigEndian(header, spi_data_size);
    spi_data_type = header[HEADER_SIZE-1];
    Serial.print("Data size: "); Serial.println(spi_data_size);
    Serial.print("Data type: "); Serial.println(spi_data_type);
}

void read_message_stm(uint8_t* msg, const uint32_t &spi_data_length){
  int del = 1;
  spi.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
  digitalWrite(SS_PIN,LOW);
  delay(del);
  spi.transfer(msg,spi_data_length);
  digitalWrite(SS_PIN,HIGH);
  delay(del);
  spi.endTransaction();
}
