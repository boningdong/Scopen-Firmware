/**
   @brief Send header via UART to STM.
   
   @return True if ACK was recieved, false if otherwise or timeout
   
   @param data_size Amount of data being sent in the body
   @param data_type Type of data being sent in the body
*/
bool send_header_stm(const uint32_t &data_size, const uint8_t &data_type){
  uint8_t header[HEADER_SIZE];
  constructHeader(header,data_size,data_type);
  Serial.println("Sending header to STM");
  Serial2.write(header,HEADER_SIZE);
  Serial2.flush();
  return wait_for_ack_stm(50);
}

/**
   @brief Send message via UART to STM.
   
   @return True if ACK was recieved, false if otherwise or timeout

   @param msg Pointer to msg being sent
   @param data_length Amount of data being sent in the body
*/
bool write_message_stm(const uint8_t* msg, const uint32_t &data_length){
  Serial.print("Sending message to STM: ");
  Serial.println(Serial2.write(msg, data_length));
  Serial.print("Message to STM: ");
  for(int i =0;i<data_length;i++){
    Serial.print(msg[i]);
    Serial.print(" ");
  }
  Serial.println("");
  Serial2.flush();
  return wait_for_ack_stm(50);
}

/**
   @brief Wait for ACK from STM.
   
   @return True if ACK was recieved, false if otherwise or timeout

   @param timeout Time in millis for timeout
*/
bool wait_for_ack_stm(int timeout){
  unsigned long sec = millis(); unsigned long current_time = millis();
  while(!(Serial2.available()>0)&& !((current_time - sec)>timeout)){
    current_time = millis();
  }
  if(current_time-sec>timeout){
    Serial.println("UART ACK timed out\n");
    return false;
  }
  if(Serial2.read() == (int) 'A'){
    Serial.println("Recieved ACK UART\n");
    return true;
  }
  else{
    Serial.println("Wrong ACK recieved from UART\n");
    return false;
  }
}

bool send_stop_command(){
  byte msg = 0xFF; bool success = false;
  if(!send_header_stm(1,CMD_STOP_SAMPLE)){
    Serial.println("STOP CMD failed");
    success = false;
  }
  if(!write_message_stm(&msg,1)){
    success = false;
  }
  return success;
}
