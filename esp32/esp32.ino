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
//software to pen
static const byte CMD_START_SAMPLE = 0x21;
static const byte CMD_STOP_SAMPLE = 0x22;
static const byte CMD_CHECK_BAT = 0x23;
static const byte CMD_SET_VOLTAGE = 0x41;
static const byte CMD_SET_SAMPLE_PARAS = 0x42;
//pen to software
static const byte CMD_DATA = 0x00;
static const byte CMD_REPORT_BAT = 0x01;
static const byte CMD_SWIPE_UP = 0x11;
static const byte CMD_SWIPE_DOWN = 0x12;
static const byte CMD_CHANGE_SEL = 0x13;

static const int MISO_PIN = 13;
static const int MOSI_PIN = 12;
static const int SCK_PIN = 14;
static const int SS_PIN =15;
static const int INTERRUPT_PIN = 23;
static const int SPI_HEADER_SIZE = 5;
static const int SPI_HEADER_SIZE_FIELD = 4;
static const int SPI_HEADER_SIZE_TYPE = 1;
static const int HEADER_SIZE = 5;
static const int HEADER_SIZE_FEILD = 4;
static const int HEADER_TYPE_FIELD = 1;
static const char SCAN_MESSAGE[] = "SCOPEN_SCAN";
char scan_send_msg[1024] = "";
IPAddress userIP;
IPAddress penIP;
uint8_t STATE = 0;
uint8_t COMM_STATE = 0;
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
      uint32_t wifiDataLength = 0; short int wifiDataType = 0;
      uint32_t spiDataLength = 0; short int spiDataType = 0;
      byte* msg = NULL;
      clientRecieve = serverRecieve.available();
      clientSend = serverSend.available();
      delay(100);
      if(clientSend&&clientRecieve){
        Serial.println("Connected");
        while(clientSend.connected()&&clientRecieve.connected()){
            if(COMM_STATE == 0){
              readHeaderSPI(spiDataLength, spiDataType);
              readHeaderWIFI(wifiDataLength, wifiDataType);
            }
            if(COMM_STATE == 1){
              msg = new byte[spiDataLength];
              readMessageSPI(msg,spiDataLength);
              writeMessageWIFI(msg,spiDataLength,spiDataType);
              delete [] msg;
              COMM_STATE == 0;
            }
            if(COMM_STATE == 2){
              msg = new byte[wifiDataLength];
              readMessageWIFI(msg, wifiDataLength);
              writeMessageSPI(msg, wifiDataLength,spiDataType);
              delete [] msg;
              COMM_STATE == 0;
            }
          delay(1);
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


void processCMDFromSTM(const short int &spiDataType){
  switch(spiDataType){
    case CMD_DATA:
      COMM_STATE = 1;
      break;
    case CMD_REPORT_BAT:
      COMM_STATE = 1;
      break;
    case CMD_SWIPE_UP:
      COMM_STATE = 0;
      break;
    case CMD_SWIPE_DOWN:
      COMM_STATE = 0;
      break;
    case CMD_CHANGE_SEL:
      COMM_STATE = 0;
      break;
    default:
      return;
  }
  sendHeaderWIFI(0,spiDataType);
}

void processCMDFromUser(const short int &dataType){
  switch(dataType){
    case CMD_START_SAMPLE:
      COMM_STATE = 0;
      break;
    case CMD_STOP_SAMPLE:
      COMM_STATE = 0;
      break;
    case CMD_CHECK_BAT:
      COMM_STATE = 0;
      break;
    case CMD_SET_VOLTAGE:
      COMM_STATE = 2;
      break;
    case CMD_SET_SAMPLE_PARAS:
      COMM_STATE = 2;
      break;
    default:
      return;
  }
  sendHeaderSPI(0,dataType);
}

void sendHeaderWIFI(const uint32_t &dataSize, const short int &dataType){
  byte header[HEADER_SIZE];
  header[3] = dataSize >> 32; 
  header[2] = dataSize >> 40; 
  header[1] = dataSize >> 48; 
  header[0] = dataSize >> 56;
  header[4] = dataType;
  clientSend.write(header,HEADER_SIZE);
  clientSend.flush();
  waitForACKWIFI();
}

void sendHeaderSPI(const uint32_t &dataSize, const short int &dataType){
  int del = 1;
  byte header[HEADER_SIZE];
  header[3] = dataSize >> 32; 
  header[2] = dataSize >> 40; 
  header[1] = dataSize >> 48; 
  header[0] = dataSize >> 56;
  header[4] = dataType;
  spi.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
  digitalWrite(SS_PIN,LOW);
  delay(del);
  spi.writeBytes(header, SPI_HEADER_SIZE);
  digitalWrite(SS_PIN,HIGH);
  delay(del);
  spi.endTransaction();
  waitForACKSPI();
}

void writeMessageWIFI(const byte* msg, const uint32_t &dataLength, const short int &msgType){
  sendHeaderWIFI(dataLength, msgType);
  Serial.println("Sending message");
  Serial.print("Sent: ");
  Serial.println(clientSend.write(msg, dataLength));
  clientSend.flush();
}

void writeMessageSPI(const byte* msg, uint32_t msgLength, short int type){
  int del = 1;
  sendHeaderSPI(msgLength, type);
  spi.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
  digitalWrite(SS_PIN,LOW);
  delay(del);
  spi.writeBytes(msg, msgLength);
  digitalWrite(SS_PIN,HIGH);
  delay(del);
  spi.endTransaction();
  waitForACKSPI();
}

void sendACKWIFI(){
  clientRecieve.write('A');
  clientRecieve.flush();
}

void sendACKSPI(){
  int del = 1;
  spi.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
  digitalWrite(SS_PIN,LOW);
  delay(del);
  spi.write('A');
  digitalWrite(SS_PIN,HIGH);
  delay(del);
  spi.endTransaction();
}

bool waitForACKWIFI(){
  while(!(clientSend.available()>0)){
  }

  if(clientSend.read() == (int)'A'){
    Serial.println("Recieved ACK");
    return true;
  }
  return false;
}

bool waitForACKSPI(){
  while(!readFlag){
  }
  readFlag = 0;
  if(spi.transfer('A') == (int)'A'){
    Serial.println("Recieved ACK");
    return true;
  }
  return false;
}



void readHeaderWIFI(uint32_t &dataSize, short int &dataType){
  if(clientRecieve.available()==HEADER_SIZE){
      byte header[HEADER_SIZE];
      Serial.print("Read: ");
      Serial.println(clientRecieve.read(header,HEADER_SIZE));  
      parseBigEndian(header,dataSize);
      dataType = header[HEADER_SIZE-1];
      Serial.print("Data size: "); Serial.println(dataSize);
      Serial.print("Data type: "); Serial.println(dataType);
      sendACKWIFI();
      processCMDFromUser(dataType);
  }
}

void readHeaderSPI(uint32_t &spiDataSize, short int &spiDataType){
  if(readFlag == 1){
    byte header[SPI_HEADER_SIZE];
    int del = 1;
    spi.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
    digitalWrite(SS_PIN,LOW);
    delay(del);
    spi.transfer(header,SPI_HEADER_SIZE);
    digitalWrite(SS_PIN,HIGH);
    delay(del);
    spi.endTransaction();
    Serial.println("SPI Read: ");
    parseBigEndian(header, spiDataSize);
    spiDataType = header[HEADER_SIZE-1];
    Serial.print("Data size: "); Serial.println(spiDataSize);
    Serial.print("Data type: "); Serial.println(spiDataType);
    sendACKSPI();
    processCMDFromSTM(spiDataType);
  }
}

void readMessageSPI(byte* msg, const uint32_t &spiDataLength){
  while(!readFlag){}
  int del = 1;
  readFlag = 1;
  spi.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
  digitalWrite(SS_PIN,LOW);
  delay(del);
  spi.transfer(msg,spiDataLength);
  digitalWrite(SS_PIN,HIGH);
  delay(del);
  spi.endTransaction();
}

void readMessageWIFI(byte* msg, uint32_t dataSize){
  while(!(clientRecieve.available()>0)){}
  Serial.println("RecieveData");
  clientRecieve.read(msg,dataSize);
//  for(int i = 0; i<dataSize; i++){
//    Serial.println(msg[i]);
//  }
  sendACKWIFI(); 
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
