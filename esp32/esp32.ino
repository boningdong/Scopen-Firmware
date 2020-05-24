/*
 *  This sketch sends a message to a TCP server
 *
 */

#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>

#define ARRAY_SIZE 400
static const uint16_t SCAN_LISTEN_PORT = 4445;
static const uint16_t SCAN_SEND_PORT = 4446;
static const uint16_t TCP_PORT = 6000;
static const char SCAN_MESSAGE[] = "SCOPEN_SCAN";
String scan_send_msg = "<0x69|1|0x69|";
IPAddress userIP;
IPAddress penIP;
uint8_t STATE = 0;
const char *ssid = "Scopen";
const char *password = "123456789";
const char * host = "192.168.4.2"; // 

WiFiUDP udp;
WiFiServer server;
WiFiClient client;
WiFiMulti WiFiMulti;
String line = "";
uint8_t dataArray[ARRAY_SIZE];

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
          scan_send_msg+=penIP+TCP_PORT+">";
          udp.beginPacket(userIP,SCAN_SEND_PORT);
          udp.println(scan_send_msg);
          udp.endPacket();
          server.begin(TCP_PORT);
          Serial.println("TCP Operation");
          STATE = 1;
        }
      }
    }
    else if(STATE == 1){
      String reciever = "";
      client = server.available();
      if(client){
        Serial.println("Connected");
        while(client.connected()){
          while(client.available()>0){
            reciever = client.readStringUntil('\n');
          }
          processCommand(reciever);
          delay(10);
        }
        client.stop();
        Serial.println("Client disconnected");
      }
    }
    
}
void processCommand(String reciever){
  reciever.replace("\n","");
  reciever.replace("\r","");
  if(reciever == "start"){
    client.println("start");
  }
  else{
    client.println("invalid");
  }
  client.flush(); 
}
void sendMode(){
//  Serial.println(client.write(dataArray,ARRAY_SIZE));
//  client.write("end\n");
//  client.flush();
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
