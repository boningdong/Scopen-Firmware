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

static const uint16_t SPI_SPEED = 10000000;
static const int MISO_PIN = 13;
static const int MOSI_PIN = 12;
static const int SCK_PIN = 14;
static const int SS_PIN = 15;
static const int INTERRUPT_PIN = 23;

const static int serial_TX_Pin = 22;
const static int serial_RX_Pin = 19;

char scan_send_msg[1024] = "";
int connection_time = 0;
int spi_recieve_timeout = 0;

uint8_t CONNECTION_STATE = 0;
const char *ssid = "Scopen";
const char *password = "123456789";
uint8_t intFlag = 0;

bool verify_cmd_from_stm(const uint8_t &data_type);
bool write_message_stm(const uint8_t* msg, const uint32_t &data_length);
void read_message_stm(uint8_t* msg, const uint32_t &spi_data_length);

bool send_header_stm(const uint32_t &data_size, const uint8_t &data_type);
void send_ack_stm();
bool wait_for_ack_stm(int timeout);
void read_header_stm(uint32_t &spi_data_Size, uint8_t &spi_data_ype);

void read_message_wifi(uint8_t* msg, const uint32_t &data_size);
void read_header_wifi(uint32_t &data_size, uint8_t &data_type);
bool send_header_wifi(const uint32_t &data_size, const uint8_t &data_type);
void send_ack_wifi();
bool wait_for_ack_wifi(int timeout);
bool write_message_wifi(const uint8_t* msg, const uint32_t &data_length);
bool verify_cmd_from_user(const uint8_t &data_type);
bool udp_listen();
void tcp_start();
void tcp_stop();
bool check_tcp_client();

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
  Serial.println("");
  Serial.println("Interrupted");
}

void upStreamTask(void* pvParameters) {
  while (true) {
    if (intFlag) {
      intFlag = 0;
      read_header_stm(incoming.dataCount, incoming.dataType);

      if (verify_cmd_from_stm(incoming.dataType)) {
        if (CONNECTION_STATE == 1)
          send_header_wifi(incoming.dataCount, incoming.dataType);

        Serial.println("Reading and sending in chunks");
        while (incoming.dataCount > 0) {
          uint32_t readLength = incoming.dataCount > MAX_SPI_BUFFER ? MAX_SPI_BUFFER : incoming.dataCount;
          read_message_stm(incoming.msg, readLength);
          if (CONNECTION_STATE == 1)
            write_message_wifi(incoming.msg, readLength); //ab
          incoming.dataCount = incoming.dataCount - readLength;
          Serial.println(incoming.dataCount);
          Serial.println("");
        }
        send_ack_stm();
        incoming.dataCount = 0;
      }
      else {
        Serial.println("Wrong header datatype sent from SPI");
        incoming.dataCount = 0;
      }
    }
    delay(1);
  }
}


void downStreamTask(void* pvParameters) {
  CONNECTION_STATE = 1;
  Serial.println("Connected");
  uint32_t dataSize; uint8_t dataType; byte msg[] = {0, 0, 0, 0, 0, 0, 0};
  while (clientSend.connected() && clientRecieve.connected()) {
    if (clientRecieve.available() == HEADER_SIZE) {
      read_header_wifi(dataSize, dataType);
      if (verify_cmd_from_user(dataType)) {
        send_ack_wifi();
        if (dataSize) {
          read_message_wifi(msg, dataSize);
          send_ack_wifi();
          send_header_stm(dataSize, dataType);
          write_message_stm(msg, dataSize);
        }
        else {
          send_header_stm(dataSize, dataType);
        }
      }
      else {
        Serial.println("Wrong header dataType recieved from WIFI");
      }
    }
  }
  Serial.println("TCP Client disconnected");
  CONNECTION_STATE = 0;
  udp.begin(SCAN_LISTEN_PORT);
  vTaskDelete(NULL);
}


void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, serial_RX_Pin, serial_TX_Pin);
  Serial.println("Creating AP ...");
  WiFi.softAP(ssid, password);
  penIP = WiFi.softAPIP();
  Serial.println("AP IP address: ");
  Serial.println(penIP);
  xTaskCreate(upStreamTask, "downStream", 10000, NULL, 2, NULL);
  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
  attachInterrupt(INTERRUPT_PIN, flagReadADCData, RISING);
  spi.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);

  WiFi.onEvent(wifi_event_handler);
  udp.begin(SCAN_LISTEN_PORT);
  tcp_start();
  delay(500);
}



void loop()
{
  if (CONNECTION_STATE == 0&&udp_listen()) {
    Serial.println("Recieved Scopen message");
    delay(10);
  }
  if (CONNECTION_STATE == 0 && check_tcp_client()) {
    udp.stop();
    xTaskCreate(downStreamTask, "downStream", 10000, NULL, 3, NULL);
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
