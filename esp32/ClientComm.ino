void readMessageWIFI(uint8_t* msg, uint32_t dataSize){
  while(!(clientRecieve.available()>0)){}
  Serial.print("Recieved WIFI Data: ");
  Serial.println(clientRecieve.read(msg,dataSize));
}

void readHeaderWIFI(uint32_t &dataSize, uint8_t &dataType){
    byte header[HEADER_SIZE];
    Serial.print("Read: ");
    Serial.println(clientRecieve.read(header,HEADER_SIZE));  
    parseBigEndian(header,dataSize);
    dataType = header[HEADER_SIZE-1];
    Serial.print("Data size: "); Serial.println(dataSize);
    Serial.print("Data type: "); Serial.println(dataType);
}

bool sendHeaderWIFI(const uint32_t &dataSize, const uint8_t &dataType){
  byte header[HEADER_SIZE];
  constructHeader(header,dataSize,dataType);
  clientSend.write(header,HEADER_SIZE);
  clientSend.flush();
  return waitForACKWIFI(10);
}

bool waitForACKWIFI(int timeout){
  unsigned long sec = millis();
  while(!(clientSend.available()>0) && !((millis()-sec)>timeout)){ //need timeout
  }

  if(clientSend.read() == (int)'A'){
    Serial.println("Recieved ACK WIFI");
    return true;
  }
  Serial.println("No ACK recieved WIFI");
  return false;
}

void sendACKWIFI(){
  clientRecieve.write('A');
  clientRecieve.flush();
}

bool writeMessageWIFI(const uint8_t* msg, const uint32_t &dataLength){
  Serial.println("Sending message");
  Serial.print("Sent: ");
  Serial.println(clientSend.write(msg, dataLength));
  clientSend.flush();
  return waitForACKWIFI(10);
}

bool verifyCMDFromUser(const uint8_t &dataType){
  return dataType == CMD_START_SAMPLE || dataType == CMD_STOP_SAMPLE
         || dataType == CMD_CHECK_BAT || dataType == CMD_SET_VOLTAGE
         || dataType == CMD_SET_SAMPLE_PARAS;
}

bool udpListen(){
  int packsize = udp.parsePacket();
  char package_buffer[256];
  if(packsize!=0){
    String msg = udp.readString();
    Serial.println(msg);
    if(msg == SCAN_MESSAGE){
      userIP = udp.remoteIP();
      Serial.print("User ip: ");
      Serial.println(userIP);
      String temp = "<105|1|105|"+penIP.toString() +'|'+TCP_PORT_RECIEVE+'|'+TCP_PORT_SEND+'>';
      temp.toCharArray(scan_send_msg,1024);
      udp.beginPacket(userIP,SCAN_SEND_PORT);
      udp.write((uint8_t*)scan_send_msg,1024);
      Serial.println(scan_send_msg);
      udp.endPacket();
      serverRecieve.begin(TCP_PORT_RECIEVE);
      serverSend.begin(TCP_PORT_SEND);
      Serial.println("TCP Operation");
      return true;
    }
  }
  return false;
}
bool tcpStart(){
  clientRecieve = serverRecieve.available();
  clientSend = serverSend.available();
  delay(10);
  return clientRecieve && clientSend;
}
void tcpStop(){
  clientRecieve.stop();
  clientSend.stop();
  Serial.println("Client disconnected");
  serverRecieve.close();
  serverSend.close();
}
