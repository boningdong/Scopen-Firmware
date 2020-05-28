/*
 *  This sketch sends a message to a TCP server
 *
 */

#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>
#include <SPI.h>

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
static const int HEADER_SIZE = 5;
static const int HEADER_SIZE_FEILD = 4;
static const int HEADER_TYPE_FIELD = 1;
static const short int MAX_SPI_BUFFER = 4000;
static const char SCAN_MESSAGE[] = "SCOPEN_SCAN";
char scan_send_msg[1024] = "";
IPAddress userIP;
IPAddress penIP;
uint8_t CONNECTION_STATE = 0;
const char *ssid = "Scopen";
const char *password = "123456789";

WiFiUDP udp;
WiFiServer serverSend;
WiFiServer serverRecieve;
WiFiClient clientSend;
WiFiClient clientRecieve;

const static int serialRXPin = 19;
const static int serialTXPin = 22;
SPIClass spi(HSPI);

static int taskCore = 0;

struct upStream {
  uint32_t dataCount;
  short int dataType;
  bool recievedHeader = false;
  byte* msg;
};
upStream incoming;
void IRAM_ATTR flagReadADCData(void){
  if(!incoming.recievedHeader){
    readHeaderSPI(incoming.dataCount, incoming.dataType);
    if(verifyCMDFromSTM(incoming.dataType)){
      incoming.recievedHeader = true;
      sendHeaderWIFI(incoming.dataCount, incoming.dataType);
      sendACKSPI();
    }
    else{
      Serial.println("Wrong header datatype sent from SPI");
      incoming.recievedHeader = false;
    }
  }
  else{
    if(incoming.dataCount>MAX_SPI_BUFFER){
      incoming.msg = new byte[MAX_SPI_BUFFER];
      readMessageSPI(incoming.msg, MAX_SPI_BUFFER);
      writeMessageWIFI(incoming.msg, MAX_SPI_BUFFER);
      delete [] incoming.msg;
      incoming.dataCount -= MAX_SPI_BUFFER;
    }
    else{
      incoming.msg = new byte[incoming.dataCount];
      readMessageSPI(incoming.msg, incoming.dataCount);
      writeMessageWIFI(incoming.msg, incoming.dataCount);
      delete [] incoming.msg;
      incoming.recievedHeader = false;
    }
    sendACKSPI();
  }
  
}

void downStreamTask(void* pvParameters){
  
  while(true){
    
  }
}

void setup()
{
    Serial.begin(115200);
    Serial2.begin(115200,SERIAL_8N1,serialRXPin,serialTXPin);
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
    xTaskCreatePinnedToCore(downStreamTask,"downStream",10000,NULL,0,NULL,taskCore);
    udp.begin(SCAN_LISTEN_PORT);
    delay(500);
}


void loop()
{
    if(CONNECTION_STATE == 0){
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
          CONNECTION_STATE = 1;
        }
      }
    }
    else if(CONNECTION_STATE == 1){
      uint32_t spiDataLength = 0; short int spiDataType = 0;
      byte* msg = NULL;
      clientRecieve = serverRecieve.available();
      clientSend = serverSend.available();
      delay(100);
      if(clientSend&&clientRecieve){
        Serial.println("Connected");
        uint32_t dataSize; short int dataType; byte* msg;
        while(clientSend.connected()&&clientRecieve.connected()){
          if(clientSend.connected() && clientRecieve.connected()){
            if(clientRecieve.available()==HEADER_SIZE){
              readHeaderWIFI(dataSize, dataType);
              if(verifyCMDFromUser(dataType)){
                sendACKWIFI();
                msg = new byte[dataSize];
                readMessageWIFI(msg,dataSize);
                sendACKWIFI();
                sendHeaderUART(dataSize, dataType);
                writeMessageUART(msg, dataSize);
                delete [] msg;
              }
              else{
                Serial.println("Wrong header dataType recieved from WIFI"); 
              }
            }
          }
        }
        clientRecieve.stop();
        clientSend.stop();
        Serial.println("Client disconnected");
        serverRecieve.close();
        serverSend.close();
        CONNECTION_STATE == 0;
      }
    } 
}


bool verifyCMDFromSTM(const short int &dataType){
  return dataType == CMD_DATA || dataType == CMD_REPORT_BAT
         || dataType == CMD_SWIPE_UP || dataType == CMD_SWIPE_DOWN
         || dataType == CMD_CHANGE_SEL;
}

bool verifyCMDFromUser(const short int &dataType){
  return dataType == CMD_START_SAMPLE || dataType == CMD_STOP_SAMPLE
         || dataType == CMD_CHECK_BAT || dataType == CMD_SET_VOLTAGE
         || dataType == CMD_SET_SAMPLE_PARAS;
}

void sendHeaderUART(const uint32_t &dataSize, const short int &dataType){
  byte header[HEADER_SIZE];
  constructHeader(header,dataSize,dataType);
  Serial2.write(header,HEADER_SIZE);
  Serial2.flush();
  waitForACKUART();
}

void sendHeaderWIFI(const uint32_t &dataSize, const short int &dataType){
  byte header[HEADER_SIZE];
  constructHeader(header,dataSize,dataType);
  clientSend.write(header,HEADER_SIZE);
  clientSend.flush();
  waitForACKWIFI();
}


void constructHeader(byte* header, const uint32_t &dataSize, const short int &dataType){
  header[3] = dataSize >> 32; 
  header[2] = dataSize >> 40; 
  header[1] = dataSize >> 48; 
  header[0] = dataSize >> 56;
  header[4] = dataType;
}

void writeMessageUART(const byte* msg, const uint32_t &dataLength){
  
  Serial.println("Sending message to STM");
  Serial.print("Sent: ");
  Serial.println(Serial2.write(msg, dataLength));
  Serial2.flush();
  waitForACKUART();
}

void writeMessageWIFI(const byte* msg, const uint32_t &dataLength){
  Serial.println("Sending message");
  Serial.print("Sent: ");
  Serial.println(clientSend.write(msg, dataLength));
  clientSend.flush();
  waitForACKWIFI();
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

bool waitForACKUART(){
  while(!(Serial2.available()>0)){} //need timeout
  if(Serial2.read() == (int) 'A'){
    Serial.println("Recieved ACK UART");
    return true;
  }
  return false;
}

bool waitForACKWIFI(){
  while(!(clientSend.available()>0)){ //need timeout
  }

  if(clientSend.read() == (int)'A'){
    Serial.println("Recieved ACK WIFI");
    return true;
  }
  return false;
}

void readHeaderWIFI(uint32_t &dataSize, short int &dataType){
    byte header[HEADER_SIZE];
    Serial.print("Read: ");
    Serial.println(clientRecieve.read(header,HEADER_SIZE));  
    parseBigEndian(header,dataSize);
    dataType = header[HEADER_SIZE-1];
    Serial.print("Data size: "); Serial.println(dataSize);
    Serial.print("Data type: "); Serial.println(dataType);
}

void readHeaderSPI(uint32_t &spiDataSize, short int &spiDataType){
    byte header[HEADER_SIZE];
    int del = 1;
    spi.beginTransaction(SPISettings(SPI_SPEED,MSBFIRST,SPI_MODE0));
    digitalWrite(SS_PIN,LOW);
    delay(del);
    spi.transfer(header,HEADER_SIZE);
    digitalWrite(SS_PIN,HIGH);
    delay(del);
    spi.endTransaction();
    Serial.println("SPI Read: ");
    parseBigEndian(header, spiDataSize);
    spiDataType = header[HEADER_SIZE-1];
    Serial.print("Data size: "); Serial.println(spiDataSize);
    Serial.print("Data type: "); Serial.println(spiDataType);
}

void readMessageSPI(byte* msg, const uint32_t &spiDataLength){
  int del = 1;
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
