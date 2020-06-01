/**
 * @brief Send header to WIFI.
 * 
 * @returns True if ACK was successfully recieved, false if otherwise or timed out.
 * 
 * @param data_size Data size of body to be sent.
 * @param data_size Data type of body to be sent.
 */
bool send_header_wifi(const uint32_t &data_size, const uint8_t &data_type) {
  assert(clientTX.connected());
  byte header[HEADER_SIZE];
  constructHeader(header, data_size, data_type);
  if (clientTX.available() > 0) {
    clientTX.flush();
  }
  clientTX.write(header, HEADER_SIZE);

  return wait_for_ack_wifi(50);
}

/**
   @brief Write message to WIFI.
   
   @return True if ACK was recieved, false if otherwise or timeout.
   
   @param msg Pointer to byte array to be sent.
   @param data_length Amount of data to be sent.
*/
bool write_message_wifi(const uint8_t* msg, const uint32_t &data_length) {
  assert(clientTX.connected());
  if (clientTX.available() > 0) {
    clientTX.flush();
  }
  clientTX.write(msg, data_length);
  return wait_for_ack_wifi(50);
}

/**
   @brief Wait for ACK from WIFI.
   
   @return True if ACK was recieved, false if otherwise or timeout
   
   @param timeout Time in milliseconds to wait for something to appear
                  in RX buffer.
*/
bool wait_for_ack_wifi(int timeout) {
  unsigned long sec = millis(); 
  unsigned long current_time = millis();
  while (!(clientTX.available() > 0) && !((current_time - sec) > timeout)) {
    current_time = millis();
  }
  
  if ((current_time - sec) >= timeout) {
    Serial.println("WIFI ACK timed out");
    return false;
  }
  if (clientTX.read() == (int)'A') {
      return true;
    }
  else {
      Serial.println("Wrong ACK recieved WIFI");
  }
  return false;
}
