#ifndef CLIENTCOMM_H
#define CLIENTCOMM_H

#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>
static const char SCAN_MESSAGE[] = "SCOPEN_SCAN";
class ClientComm : public Comm{
  public:
    ClassComm();
    bool clientListen();
    void readHeader(uint32_t &dataSize, short int &dataType);
    void readMsg (short int* msg, uint32_t dataSize);
    void sendACK();
    bool writeMsg();
  
  private:
    bool waitForACK();
    void wifi_event_handler(WiFiEvent_t event);
    bool verifyCMDFromUser(const short int &dataType);
    const uint16_t SCAN_LISTEN_PORT;
    const uint16_t SCAN_SEND_PORT;
    const uint16_t TCP_PORT_SEND;
    const uint16_t TCP_PORT_RECIEVE;
    const char *ssid = "Scopen";
    const char *password = "123456789";
    char scan_send_msg[1024] = "";
    IPAddress userIP;
    IPAddress penIP;

    WiFiUDP udp;
    WiFiServer serverSend;
    WiFiServer serverRecieve;
    WiFiClient clientSend;
    WiFiClient clientRecieve;
    
};
ClientComm::SCAN_LISTEN_PORT = 4445;
ClientComm::SCAN_SEND_PORT = 4446;
ClientComm::TCP_PORT_SEND = 6000;
ClientComm::TCP_PORT_RECIEVE = 7000;

#endif
