/*
    This sketch sends a message to a TCP server

*/
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>
#include <stdint.h>
#include <SPI.h>

//software to pen
static const uint8_t CMD_START_SAMPLE = 0x21;
static const uint8_t CMD_STOP_SAMPLE = 0x22;
static const uint8_t CMD_CHECK_BAT = 0x23;
static const uint8_t CMD_SET_VOLTAGE = 0x41;
static const uint8_t CMD_SET_SAMPLE_PARAS = 0x42;
//pen to software
static const uint8_t CMD_DATA = 0x00;
static const uint8_t CMD_REPORT_BAT = 0x01;
static const uint8_t CMD_SWIPE_UP = 0x11;
static const uint8_t CMD_SWIPE_DOWN = 0x12;
static const uint8_t CMD_CHANGE_SEL = 0x13;
//header definition
static const int HEADER_SIZE = 5;
static const int HEADER_SIZE_FEILD = 4;
static const int HEADER_TYPE_FIELD = 1;

static const uint16_t SCAN_LISTEN_PORT = 4445;
static const uint16_t SCAN_SEND_PORT = 4446;
static const uint16_t TCP_PORT_SEND = 6000;
static const uint16_t TCP_PORT_RECIEVE = 7000;

static const short int MAX_SPI_BUFFER = 4096;
static const char SCAN_MESSAGE[] = "SCOPEN_SCAN";
static const int CONNECTION_TIMEOUT = 2;
static const int SPI_CONNECTION_TIMEOUT = 10;

static const uint16_t SPI_SPEED = 10000000;
static const int MISO_PIN = 13;
static const int MOSI_PIN = 12;
static const int SCK_PIN = 14;
static const int SS_PIN = 15;
static const int INTERRUPT_PIN = 23;

const static int serialTXPin = 22;
const static int serialRXPin = 19;

char scan_send_msg[1024] = "";
int connection_time = 0;
int spi_recieve_timeout = 0;

uint8_t CONNECTION_STATE = 0;
const char *ssid = "Scopen";
const char *password = "123456789";
uint8_t intFlag = 0;

bool verifyCMDFromSTM(const uint8_t &dataType);
bool writeMessageSTM(const uint8_t* msg, const uint32_t &dataLength);
void readMessageSTM(uint8_t* msg, const uint32_t &spiDataLength);

bool sendHeaderSTM(const uint32_t &dataSize, const uint8_t &dataType);
void sendACKSTM();
bool waitForACKSTM(int timeout);
void readHeaderSTM(uint32_t &spiDataSize, uint8_t &spiDataType);

void readMessageWIFI(uint8_t* msg, const uint32_t &dataSize);
void readHeaderWIFI(uint32_t &dataSize, uint8_t &dataType);
bool sendHeaderWIFI(const uint32_t &dataSize, const uint8_t &dataType);
void sendACKWIFI();
bool waitForACKWIFI(int timeout);
bool writeMessageWIFI(const uint8_t* msg, const uint32_t &dataLength);
bool verifyCMDFromUser(const uint8_t &dataType);
bool udpListen();
void tcpStart();
void tcpStop();
bool checkTCPClient();

struct upStream {
  uint32_t dataCount = 0;
  uint8_t dataType = 0;
  uint8_t msg[MAX_SPI_BUFFER];
};
upStream incoming;

WiFiUDP udp;
WiFiServer serverSend;
WiFiServer serverRecieve;
WiFiClient clientSend;
WiFiClient clientRecieve;
IPAddress userIP;
IPAddress penIP;
SPIClass spi(HSPI);

void IRAM_ATTR flagReadADCData(void) {
  intFlag = 1;
  Serial.println("Triggered");
  if (incoming.dataCount > 0) {
    if ((millis() - spi_recieve_timeout) > SPI_CONNECTION_TIMEOUT * 1000) {
      incoming.dataCount = 0;
    }
    else {
      intFlag = 0;
    }
  }
}

void upStreamTask(void* pvParameters) {
  while (true) {

    if (intFlag) {
      intFlag = 0;
      spi_recieve_timeout = 0;
      if (!(incoming.dataCount > 0)) {
        readHeaderSTM(incoming.dataCount, incoming.dataType);
        if (verifyCMDFromSTM(incoming.dataType)) {
          sendACKSTM();
          if (CONNECTION_STATE == 1)
            sendHeaderWIFI(incoming.dataCount, incoming.dataType);
        }
        if (incoming.dataCount > 0) {
          spi_recieve_timeout = millis();
        }
        else {
          Serial.println("Wrong header datatype sent from SPI");
          incoming.dataCount = 0;
        }
      }
      while (verifyCMDFromSTM(incoming.dataType)&&incoming.dataCount > 0) {
        uint32_t readLength = incoming.dataCount > MAX_SPI_BUFFER ? MAX_SPI_BUFFER : incoming.dataCount;
        readMessageSTM(incoming.msg, readLength);
        if (CONNECTION_STATE == 1)
          writeMessageWIFI(incoming.msg, readLength);
        incoming.dataCount = incoming.dataCount - readLength;
        Serial.println(incoming.dataCount);
        //        for(int i = 0; i < readLength; i++) {
        //          Serial.print(incoming.msg[i], HEX);
        //          Serial.print(' ');
        //          incoming.msg[i] = 0;
        //        }
        //        Serial.println("");
        sendACKSTM();
        //Serial.println(incoming.dataCount);
        delay(1);
      }
      incoming.dataCount = 0;
      //        incoming.recievedHeader = false;
    }
    delay(2);
  }
  
}



void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, serialRXPin, serialTXPin);
  Serial.println("Creating AP ...");
  WiFi.softAP(ssid, password);
  penIP = WiFi.softAPIP();
  Serial.println("AP IP address: ");
  Serial.println(penIP);
  xTaskCreatePinnedToCore(upStreamTask, "downStream", 10000, NULL, 0, NULL, 0);
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
  attachInterrupt(INTERRUPT_PIN, flagReadADCData, RISING);
  spi.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);

  WiFi.onEvent(wifi_event_handler);
  udp.begin(SCAN_LISTEN_PORT);
  tcpStart();
  delay(500);
}

void loop()
{
  if (udpListen()) {
    Serial.println("Recieved Scopen message");
    delay(10);
  }
  if (checkTCPClient()) {
    CONNECTION_STATE = 1;
    Serial.println("Connected");
    uint32_t dataSize; uint8_t dataType; byte msg[] = {0, 0, 0, 0, 0, 0, 0};
    while (clientSend.connected() && clientRecieve.connected()) {
      if (clientRecieve.available() == HEADER_SIZE) {
        readHeaderWIFI(dataSize, dataType);
        if (verifyCMDFromUser(dataType)) {
          sendACKWIFI();
          if (dataSize) {
            readMessageWIFI(msg, dataSize);
            sendACKWIFI();
            sendHeaderSTM(dataSize, dataType);
            writeMessageSTM(msg, dataSize);
          }
          else {
            sendHeaderSTM(dataSize, dataType);
          }
        }
        else {
          Serial.println("Wrong header dataType recieved from WIFI");
        }
      }
    }
    Serial.println("TCP Client disconnected");
  }
  else {
    CONNECTION_STATE = 0;
  }
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
