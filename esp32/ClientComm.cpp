#include "ClientComm.h"
ClientComm::ClientComm{
  
}
bool ClientComm::clientListen(){
  udp.begin(SCAN_LISTEN_PORT);
  int packsize = udp.parsePacket();
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

void ClientComm::readHeader(uint32_t &dataSize, short int &dataType){
    assert(clientRecieve.connected());
    byte header[HEADER_SIZE];
    Serial.print("Read: ");
    Serial.println(clientRecieve.read(header,HEADER_SIZE));  
    parseBigEndian(header,dataSize);
    dataType = header[HEADER_SIZE-1];
    Serial.print("Data size: "); Serial.println(dataSize);
    Serial.print("Data type: "); Serial.println(dataType);
}

void ClientComm::readMsg(short int* msg, uint32_t dataSize){
  assert(clientRecieve.connected());
  while(!(clientRecieve.available()>0)){}
  Serial.println("RecieveData");
  clientRecieve.read(msg,dataSize);
}

void ClientComm::sendACK(){
  assert(clientRecieve.connected());
  clientRecieve.write('A');
  clientRecieve.flush();
}


bool CientComm::writeMsg(const short int* msg, const uint32_t &dataLength){
  assert(clientSend.connected());
  Serial.println("Sending message");
  Serial.print("Sent: ");
  Serial.println(clientSend.write(msg, dataLength));
  clientSend.flush();
  return waitForACKWIFI();
}

bool ClientComm::waitForACK(){
  while(!(clientSend.available()>0)){ //need timeout
  }

  if(clientSend.read() == (int)'A'){
    Serial.println("Recieved ACK WIFI");
    return true;
  }
  return false;
}

void ClientComm::sendHeaderWIFI(const uint32_t &dataSize, const short int &dataType){
  byte header[HEADER_SIZE];
  constructHeader(header,dataSize,dataType);
  clientSend.write(header,HEADER_SIZE);
  clientSend.flush();
  waitForACKWIFI();
}


bool ClientComm::verifyCMDFromUser(const short int &dataType){
  return dataType == CMD_START_SAMPLE || dataType == CMD_STOP_SAMPLE
         || dataType == CMD_CHECK_BAT || dataType == CMD_SET_VOLTAGE
         || dataType == CMD_SET_SAMPLE_PARAS;
}


void ClientComm::wifi_event_handler(WiFiEvent_t event)
{
  Serial.println("[WiFi Event]");
  switch (event)
  {
  case SYSTEM_EVENT_AP_START:
    Serial.println("WiFi access point started.");
    break;
  case SYSTEM_EVENT_AP_STACONNECTED:
    Serial.println("Client connected.\n");

  }
}
