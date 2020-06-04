/**
 * @brief Reads message from WIFI.
 * 
 * @param msg Pointer to the msg buffer.
 * @param data_size Size in bytes of message to be read.
 */
void read_message_wifi(uint8_t* msg, const uint32_t& data_size) {
  clientRX.read(msg, data_size);
  clientRX.flush();
}

/**
 * @brief Reads header from WIFI.
 * 
 * @param data_size Reference to store data size.
 * @param data_size Reference to store data type.
 */
void read_header_wifi(uint32_t &data_size, uint8_t &data_type) {
  byte header[HEADER_SIZE];
  clientRX.read(header, HEADER_SIZE);
  parseBigEndian(header, data_size);
  data_type = header[HEADER_SIZE - 1];
//  Serial.print("WIFI Header: ");
//  for (int i = 0; i < 5; i++) {
//    Serial.print(header[i]); Serial.print(" ");
//  }
//  Serial.println("");
  clientRX.flush();
//  Serial.print("Data size: "); Serial.println(data_size);
//  Serial.print("Data type: "); Serial.println(data_type);
}


/**
 * @brief Send ACK to WIFI.
 */
void send_ack_wifi() {
  clientRX.write('A');
}

/**
 * @brief Verifies if incoming command is valid.
 * 
 * @param data_type Command to be checked.
 */
bool verify_cmd_from_user(const uint8_t &data_type) {
  return data_type == CMD_START_SAMPLE || data_type == CMD_STOP_SAMPLE
         || data_type == CMD_CHECK_BAT || data_type == CMD_SET_VOLTAGE
         || data_type == CMD_SET_SAMPLE_PARAS;
}

/**
 * @brief Waits for something in RX buffer.
 * 
 * return True if something is RX buffer, false if timed out
 * 
 * @param timeout Time in millis to wait.
*/
bool wifi_timeout(int timeout) {
  unsigned long sec = millis(); unsigned long current_time = millis();
  while (!(clientRX.available() > 0) && !((current_time - sec) > timeout)) {
    current_time = millis();
  }
  if (clientRX.available() > 0) {
    return true;
  }
  return false;
}

bool wifi_timeout_check_size(int timeout, uint32_t dataSize) {
  unsigned long sec = millis(); unsigned long current_time = millis();
  while (!(clientRX.available() >= dataSize) && !((current_time - sec) > timeout)) {
    current_time = millis();
  }
  if (clientRX.available() >= dataSize) {
    return true;
  }
  return false;
}
