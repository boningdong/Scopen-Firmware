/**
   @brief Checks if UDP RX port received the SCAN_MESSAGE.

   @return True if SCAN_MESSAGE was recieved in UDP port
*/
void udp_listen() {

}

/**
   @brief Starts the TCP RX and TX sockets.
*/
void tcp_start() {
  serverRX.begin(TCP_PORT_RX);
  serverTX.begin(TCP_PORT_TX);
  //  Serial.println("TCP Sockets enabled");
}

/**
   @brief Stops the TCP RX and TX sockets.
*/
void tcp_stop() {
  clientRX.stop();
  clientTX.stop();
  //  Serial.println("TCP Socket stopped");
  serverRX.close();
  serverTX.close();
}

/**
   @brief Checks if someone connected to TCP RX and TX sockets.
*/
bool check_tcp_client() {
  clientRX = serverRX.available();
  clientTX = serverTX.available();
  delay(100);
  return clientRX && clientTX;
}
