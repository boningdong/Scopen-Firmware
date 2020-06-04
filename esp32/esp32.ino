/*
    This sketch sends a message to a TCP server

*/
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>
#include <stdint.h>
#include <SPI.h>

// #define SPI_DEBUG

//SPI Pins
static const int MISO_PIN = 13;
static const int MOSI_PIN = 12;
static const int SCK_PIN = 14;
static const int SS_PIN = 15;
static const int INTERRUPT_PIN = 23;

//UART pins
const static int serial_TX_Pin = 22;
const static int serial_RX_Pin = 19;

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

//WIFI AP settings
const char *ssid = "Scopen";
const char *password = "123456789";

//Ports for UDP and TCP connections
static const uint16_t SCAN_LISTEN_PORT = 4445;
static const uint16_t SCAN_REPLY_PORT = 4446;
static const uint16_t TCP_PORT_TX = 6000;
static const uint16_t TCP_PORT_RX = 7000;

//Scan message for UDP
static const char SCAN_MESSAGE[] = "SCOPEN_SCAN";
char scan_send_msg[1024] = "";

//SPI settings
static const uint16_t MAX_SPI_BUFFER = 4096;
static const uint32_t SPI_SPEED = 10000000;

//UART buffer
static const uint8_t MAX_UART_BUFFER = 16;


//ClientComm functions
bool udp_listen();
bool check_tcp_client();
void tcp_start();
void tcp_stop();

//ClientCommRX functions
bool wifi_timeout(int timeout);
bool verify_cmd_from_user(const uint8_t &data_type);
void read_message_wifi(uint8_t* msg, const uint32_t &data_size);
void read_header_wifi(uint32_t &data_size, uint8_t &data_type);
void send_ack_wifi();

//ClientCommTX functions
bool send_header_wifi(const uint32_t &data_size, const uint8_t &data_type);
bool write_message_wifi(const uint8_t* msg, const uint32_t &data_length);
bool wait_for_ack_wifi(int timeout);

//STMCommRX
bool verify_cmd_from_stm(const uint8_t &data_type);
void read_header_stm(uint32_t &spi_data_Size, uint8_t &spi_data_ype);
void read_message_stm(uint8_t* msg, const uint32_t &spi_data_length);
void send_ack_stm();

//STMCommTX
bool write_message_stm(const uint8_t* msg, const uint32_t &data_length);
bool send_header_stm(const uint32_t &data_size, const uint8_t &data_type);
bool wait_for_ack_stm(int timeout);

//upStream dataholder
struct upStream {
  uint32_t data_left = 0;
  uint8_t data_type = 0;
  uint8_t msg[MAX_SPI_BUFFER];
};
upStream incoming;

struct downStream {
  uint32_t data_left = 0;
  uint8_t data_type = 0;
  uint8_t msg[MAX_UART_BUFFER];
};
downStream down_stream;

WiFiUDP udp;
WiFiServer serverTX;
WiFiServer serverRX;
WiFiClient clientTX;
WiFiClient clientRX;
IPAddress userIP;
IPAddress penIP;

SPIClass spi(HSPI);

TaskHandle_t downStream_handle;

bool downStream = false;
bool isConnected = false;
bool udpOn = false;
uint8_t ack_failed_count = 0;
uint8_t interrupt_flag = 0;

void IRAM_ATTR flagReadADCData(void) {
  interrupt_flag = 1;
//  Serial.println("");
//  Serial.println("Interrupted");
}

void upStreamTask(void* pvParameters) {
  bool headerFailed = false;
  while (true) {
    if (interrupt_flag) {
      interrupt_flag = 0;
      read_header_stm(incoming.data_left, incoming.data_type);

      if (verify_cmd_from_stm(incoming.data_type)) {
        if (isConnected && clientTX.connected()){
          if(send_header_wifi(incoming.data_left, incoming.data_type)){
            headerFailed = false;
          }
          else{
            headerFailed = true;
          }
        }
        else{
          isConnected = false;
        }
        #ifdef SPI_DEBUG
//        Serial.println("Reading and sending in chunks");
        #endif
        while (incoming.data_left > 0) {
          uint32_t readLength = incoming.data_left > MAX_SPI_BUFFER ? MAX_SPI_BUFFER : incoming.data_left;
          #ifdef SPI_DEBUG
//          Serial.print("Data left: ");
//          Serial.print(incoming.data_left);
//          Serial.println("");
//          Serial.print("Read Length: ");
//          Serial.print(readLength);
//          Serial.println("\n");
          #endif
          read_message_stm(incoming.msg, readLength);
          if (isConnected && clientTX.connected()&&!headerFailed){
            write_message_wifi(incoming.msg, readLength);
          }
          else{
            isConnected = false;
          }
          
          incoming.data_left = incoming.data_left - readLength;

        }
        send_ack_stm();
        incoming.data_left = 0;
      }
      else {
//        Serial.println("Wrong header datatype sent from SPI");
        incoming.data_left = 0;
      }
    }
    delay(1);
  }
}

void downStreamTask(void* pvParameters) {
  while (isConnected && clientRX.connected()) {
    if (clientRX.available() == HEADER_SIZE) {
      read_header_wifi(down_stream.data_left, down_stream.data_type);

      if (verify_cmd_from_user(down_stream.data_type)) {
        send_ack_wifi();
        if (down_stream.data_left) {
          if (wifi_timeout_check_size(20,down_stream.data_left)) {
            read_message_wifi(down_stream.msg, down_stream.data_left);
            send_ack_wifi();
            send_header_stm(down_stream.data_left, down_stream.data_type);
            write_message_stm(down_stream.msg, down_stream.data_left);
          }
          else{
            send_ack_wifi();
            if(clientRX.available() > 0){
              clientRX.flush();
            }
          }
        }
      }
      else {
        Serial.println("Wrong header dataType recieved from WIFI");
        send_ack_wifi();
      }
    }
  }

//  Serial.println("TCP Client disconnected");
  isConnected = false;
  downStream = false;
  vTaskDelete(downStream_handle);
}


void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, serial_RX_Pin, serial_TX_Pin);
  //wifi_power_t power;
//  Serial.println("Creating AP ...");
  WiFi.softAP(ssid, password);
  penIP = WiFi.softAPIP();
  WiFi.setTxPower(WIFI_POWER_11dBm);
//  Serial.println("AP IP address: ");
//  Serial.print(penIP);

  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, HIGH);
  attachInterrupt(INTERRUPT_PIN, flagReadADCData, RISING);

  spi.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);

  WiFi.onEvent(wifi_event_handler);
  xTaskCreate(upStreamTask, "downStream", 10000, NULL, 2, NULL);
  udp.begin(SCAN_LISTEN_PORT);
//  Serial.println("UDP ENABLED");
  udpOn = true;
  tcp_start();
  delay(500);
}



void loop()
{
  if(!isConnected && !udpOn ){
    udp.begin(SCAN_LISTEN_PORT);
    udpOn = true;
  }
  if (!isConnected && udp_listen()) {
//    Serial.println("Recieved Scopen message");
    delay(10);
  }
  if (!isConnected && check_tcp_client()) {
    udp.stop();
    udpOn = false;
//    Serial.println("Connected");
    isConnected = true;
    if (downStream)
      vTaskDelete(downStream_handle);
    xTaskCreate(downStreamTask, "downStream", 10000, NULL, 3, &downStream_handle);
    downStream = true;
  }
}

void wifi_event_handler(WiFiEvent_t event)
{
//  Serial.println("[WiFi Event]");
  switch (event)
  {
    case SYSTEM_EVENT_AP_START:
//      Serial.println("WiFi access point started.");
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
//      Serial.println("WIFI Client connected.");
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
//      Serial.println("WIFI Client disconnected.");
      isConnected = false;
      break; 
  }
//  Serial.println("");
}
