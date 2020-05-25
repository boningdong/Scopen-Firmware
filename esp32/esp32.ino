/*
 *  This sketch sends a message to a TCP server
 *
 */

#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>
#include <SPI.h>
#define ARRAY_SIZE 400
static const uint16_t SCAN_LISTEN_PORT = 4445;
static const uint16_t SCAN_SEND_PORT = 4446;
static const uint16_t TCP_PORT_SEND = 6000;
static const uint16_t TCP_PORT_RECIEVE = 7000;
static const uint16_t SPI_SPEED = 1000000;
static const int MISO_PIN = 13;
static const int MOSI_PIN = 12;
static const int SCK_PIN = 14;
static const int SS_PIN =15;
static const int INTERRUPT_PIN = 23;
static const int HEADER_SIZE = 5;
static const int HEADER_SIZE_FEILD = 4;
static const int HEADER_TYPE_FIELD = 1;
static const char SCAN_MESSAGE[] = "SCOPEN_SCAN";
char scan_send_msg[1024] = "";
IPAddress userIP;
IPAddress penIP;
uint8_t STATE = 0;
const char *ssid = "Scopen";
const char *password = "123456789";

WiFiUDP udp;
WiFiServer serverSend;
WiFiServer serverRecieve;
WiFiClient clientSend;
WiFiClient clientRecieve;

String line = "";
uint8_t dataArray[ARRAY_SIZE];
uint8_t readFlag = 0;
SPIClass spi(HSPI);

void IRAM_ATTR flagReadADCData(void){
  readFlag = 1;
}

void setup()
{
    for(int i = 0; i < ARRAY_SIZE ; i++){
      dataArray[i] = 121;
    }
    
    Serial.begin(115200);
    Serial.println("Creating AP ...");
    WiFi.softAP(ssid, password);
    penIP = WiFi.softAPIP();
    Serial.println("AP IP address: ");
    Serial.println(penIP);

    pinMode(SS_PIN, OUTPUT);
    digitalWrite(SS_PIN, HIGH);
    attachInterrupt(INTERRUPT_PIN, flagReadADCData,RISING);
    spi.begin(SCK_PIN,MISO_PIN,MOSI_PIN,SS_PIN);
    
    WiFi.onEvent(wifi_event_handler);
    udp.begin(SCAN_LISTEN_PORT);
    delay(500);
}


void loop()
{
    if(STATE == 0){
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
          STATE = 1;
        }
      }
    }
    else if(STATE == 1){
      String reciever = "";
      bool sent = true;
      uint32_t dataLength = 0; short int dataType = 0;
      clientRecieve = serverRecieve.available();
      clientSend = serverSend.available();
      delay(100);
      if(clientSend&&clientRecieve){
        Serial.println("Connected");
        byte temp[10] = {1,20,66,12,11,4,9,0,99};
        while(clientSend.connected()&&clientRecieve.connected()){
//          processHeader(dataLength, dataType);
//          if(STATE == 2 && clientRecieve.available()>0){
//            verifyIncomingMessage(dataLength);
//          }
          if(sent){
            sendMessageToClient(temp,10,1);
            sent = false;
          }        
          delay(10);
        }
        clientRecieve.stop();
        clientSend.stop();
        Serial.println("Client disconnected");
        serverRecieve.close();
        serverSend.close();
        STATE == 0;
      }
    } 
}

void sendHeader(const uint32_t &dataSize, const short int &dataType){
  byte header[HEADER_SIZE];
  header[3] = dataSize >> 32; 
  header[2] = dataSize >> 40; 
  header[1] = dataSize >> 48; 
  header[0] = dataSize >> 56;
  header[4] = dataType;
  clientSend.write(header,HEADER_SIZE);
  clientSend.flush();

}
void sendMessageToClient(byte msg[], const uint32_t &dataLength, const short int &msgType){
  sendHeader(dataLength, msgType);
  if(waitForACK()){
    Serial.println("Sending message");
    Serial.print("Sent: ");
    Serial.println(clientSend.write(msg, dataLength));
    clientSend.flush();
  }
}
bool waitForACK(){
  while(!(clientSend.available()>0)){
  }

  if(clientSend.read() == (int)'A'){
    Serial.println("Recieved ACK");
    return true;
  }
  return false;
}
void readSTM32SPI(){
  int del = 1;
  spi.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
  digitalWrite(SS_PIN,LOW);
  delay(del);
  spi.transfer(dataArray,ARRAY_SIZE);
  digitalWrite(SS_PIN,HIGH);
  delay(del);
  spi.endTransaction();
}

void writeSTM32SPI(uint8_t cmd){
  int del = 1;
  spi.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
  digitalWrite(SS_PIN,LOW);
  delay(del);
  spi.write(cmd);
  digitalWrite(SS_PIN,HIGH);
  delay(del);
  spi.endTransaction();
}

void sendAck(){
  clientRecieve.write('A');
  clientRecieve.flush();
}

bool verifyIncomingMessage(uint32_t dataSize){
  Serial.println("processMSG");
  bool sizeCorrect = false;
  byte msg[dataSize];
  if(clientRecieve.available()==dataSize){
    sizeCorrect = true;
  }
  Serial.println("RecieveData");
  clientRecieve.read(msg,dataSize);
  for(int i = 0; i<dataSize; i++){
    Serial.println(msg[i]);
  }
  sendAck(); 
  STATE = 1;
  return sizeCorrect;
}

void processHeader(uint32_t &dataSize, short int &dataType){
  byte header[HEADER_SIZE];
  if(clientRecieve.available()==HEADER_SIZE){
      Serial.print("Read: ");
      Serial.println(clientRecieve.read(header,HEADER_SIZE));  
      parseBigEndian(header,dataSize);
      dataType = header[HEADER_SIZE-1];
      Serial.print("Data size: "); Serial.println(dataSize);
      Serial.print("Data type: "); Serial.println(dataType);
      sendAck();
      STATE = 2;
  }
}
void printLongLongInt(uint32_t in){
  char buf[50];
  if(in > 0xFFFFFFFFLL) {
    sprintf(buf, "%lX%08lX", (unsigned long)(in>>32), (unsigned long)(in&0xFFFFFFFFULL));
  } else {
    sprintf(buf, "%lX", (unsigned long)in);
  }
  Serial.println( buf );
}


void parseBigEndian(byte *input, uint32_t &output){
  output =(input[4] << 24) | (input[5] << 16) | (input[6]<<8) | (input[7]);
}

void wifi_event_handler(WiFiEvent_t event)
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
