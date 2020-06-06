/**
 * @brief Checks if UDP RX port received the SCAN_MESSAGE.
 * 
 * @return True if SCAN_MESSAGE was recieved in UDP port
 */
bool udp_listen() {
  int packsize = udp.parsePacket();
  char package_buffer[256];
  if (packsize != 0) {
    String msg = udp.readString();
//    Serial.println(msg);
    if (msg == SCAN_MESSAGE) {
      userIP = udp.remoteIP();
//      Serial.print("User ip: ");
//      Serial.println(userIP);
      String temp = "<105|1|105|" + penIP.toString() + '|' + TCP_PORT_RX + '|' + TCP_PORT_TX + '>';
      temp.toCharArray(scan_send_msg, 1024);
      udp.beginPacket(userIP, SCAN_REPLY_PORT);
      udp.write((uint8_t*)scan_send_msg, 1024);
      Serial.println(scan_send_msg);
      udp.endPacket();
      return true;
    }
  }
  return false;
}

/**
 * @brief Starts the TCP RX and TX sockets.
 */
void tcp_start() {
  serverRX.begin(TCP_PORT_RX);
  serverTX.begin(TCP_PORT_TX);
//  Serial.println("TCP Sockets enabled");
}

/**
 * @brief Stops the TCP RX and TX sockets.
 */
void tcp_stop() {
  clientRX.stop();
  clientTX.stop();
//  Serial.println("TCP Socket stopped");
  serverRX.close();
  serverTX.close();
}

/**
 * @brief Checks if someone connected to TCP RX and TX sockets.
 */
bool check_tcp_client() {
  clientRX = serverRX.available();
  clientTX = serverTX.available();
  delay(100);
  return clientTX && clientTX;
}
