/**
   @brief Read header from STM.
  
   @param spi_data_size Reference to data size
   @param spi_data_type Reference to data type
*/
void read_header_stm(uint32_t &spi_data_size, uint8_t &spi_data_type){
    uint8_t header[HEADER_SIZE];
    
    spi.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
    
    digitalWrite(SS_PIN,LOW);
    spi.transferBytes(NULL,header,HEADER_SIZE);
    digitalWrite(SS_PIN,HIGH);
    
    spi.endTransaction();
    
    Serial.println(" ");
    Serial.print("SPI Header: ");
    for(int i = 0; i<5;i++){
       Serial.print(header[i]);Serial.print(" ");
    }
     Serial.println("");
    parseBigEndian(header, spi_data_size);
    spi_data_type = header[HEADER_SIZE-1];
//    Serial.print("Data size: "); Serial.println(spi_data_size);
//    Serial.print("Data type: "); Serial.println(spi_data_type);
}

/**
   @brief Read message from STM.
   
   @param msg Pointer to data to send
   @param spi_data_length Length of data to be sent
*/
void read_message_stm(uint8_t* msg, const uint32_t &spi_data_length){
  spi.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
  
  digitalWrite(SS_PIN,LOW);
  spi.transfer(msg,spi_data_length);
  digitalWrite(SS_PIN,HIGH);

  spi.endTransaction();
}

/**
   @brief Send ACK to STM via SPI.
   
*/
void send_ack_stm(){
  spi.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
  
  digitalWrite(SS_PIN,LOW);
  spi.write('A');
  digitalWrite(SS_PIN,HIGH);
  
  spi.endTransaction();
  Serial.println("Sent ACK to STM");
}

/**
   @brief Verify if command from STM is valid.

   @return True if valid false if otherwise
   
   @param spi_data_type Command to check
*/
bool verify_cmd_from_stm(const uint8_t &data_type){
  return data_type == CMD_DATA || data_type == CMD_REPORT_BAT
         || data_type == CMD_SWIPE_UP || data_type == CMD_SWIPE_DOWN
         || data_type == CMD_CHANGE_SEL;
}
