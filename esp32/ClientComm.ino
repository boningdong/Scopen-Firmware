void read_message_wifi(uint8_t* msg, const uint32_t& data_size){
  while(!(clientRecieve.available()>0)){}
//  Serial.print("Recieved WIFI Data: ");
  clientRecieve.read(msg,data_size);
}

void read_header_wifi(uint32_t &data_size, uint8_t &data_type){
    byte header[HEADER_SIZE];
    clientRecieve.read(header,HEADER_SIZE);  
    parseBigEndian(header,data_size);
    data_type = header[HEADER_SIZE-1];
//    Serial.print("Data size: "); Serial.println(data_size);
//    Serial.print("Data type: "); Serial.println(data_type);
}

bool send_header_wifi(const uint32_t &data_size, const uint8_t &data_type){
assert(clientSend.connected());
  byte header[HEADER_SIZE];
  constructHeader(header,data_size,data_type);
  clientSend.write(header,HEADER_SIZE);
  clientSend.flush();
  return wait_for_ack_wifi(20);
}

bool wait_for_ack_wifi(int timeout){
  unsigned long sec = millis(); unsigned long current_time = millis();
//  Serial.print("sec: "); Serial.println(sec);
  while(!(clientSend.available()>0) && !((current_time-sec)>timeout)){
    current_time = millis();
  }
  Serial.println("after");
  if((current_time-sec)>=timeout){
    Serial.println("WIFI ACK timed out");
    return false;
  }
  else if(clientSend.read() == (int)'A'){
    Serial.println("here");
    return true;
  }
  else{
    Serial.println("Wrong ACK recieved WIFI");
    return false;
  }
  
}

void send_ack_wifi(){
  clientRecieve.write('A');
  clientRecieve.flush();
}

bool write_message_wifi(const uint8_t* msg, const uint32_t &data_length){
assert(clientSend.connected());
//  Serial.println("Sending WIFI message");
//  Serial.print("Sent: ");
  clientSend.write(msg, data_length);
  clientSend.flush();
  return wait_for_ack_wifi(20);
}

bool verify_cmd_from_user(const uint8_t &data_type){
  return data_type == CMD_START_SAMPLE || data_type == CMD_STOP_SAMPLE
         || data_type == CMD_CHECK_BAT || data_type == CMD_SET_VOLTAGE
         || data_type == CMD_SET_SAMPLE_PARAS;
}

bool udp_listen(){
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
      return true;
    }
  }
  return false;
}

void tcp_start(){
  serverRecieve.begin(TCP_PORT_RECIEVE);
  serverSend.begin(TCP_PORT_SEND);
  Serial.println("TCP Sockets enabled");
}

bool check_tcp_client(){
  clientRecieve = serverRecieve.available();
  clientSend = serverSend.available();
  delay(100);
  return clientRecieve && clientSend;
}

void tcp_stop(){
  clientRecieve.stop();
  clientSend.stop();
  Serial.println("TCP Socket stopped");
  serverRecieve.close();
  serverSend.close();
}
