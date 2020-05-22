/*
 *  This sketch sends a message to a TCP server
 *
 */

#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiAP.h>

const char *ssid = "Scopen";
const char *password = "123456789";
//unsigned int port = 2333;
const uint16_t port = 6000;
const char * host = "192.168.4.2"; // ip or dns
char package_buffer[256];
char reply_buffer[] = "ACK";
// Use WiFiClient class to create TCP connections
WiFiClient client;

WiFiMulti WiFiMulti;

void setup()
{
    Serial.begin(115200);
    delay(10);
    Serial.println("Creating AP ...");
    WiFi.softAP(ssid, password);
    IPAddress ip = WiFi.softAPIP();
    Serial.println("AP IP address: ");
    Serial.println(ip);
    WiFi.onEvent(wifi_event_handler);

    // We start by connecting to a WiFi network
    /*WiFiMulti.addAP("SSID", "passpasspass");

    Serial.println();
    Serial.println();
    Serial.print("Waiting for WiFi... ");

    while(WiFiMulti.run() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }*/

    

    delay(500);
}


void loop()
{
//    const uint16_t port = 80;
//    const char * host = "192.168.1.1"; // ip or dns
   

    // This will send a request to the server
    //uncomment this line to send an arbitrary string to the server
    //client.print("Send this data to the server");
    //uncomment this line to send a basic document request to the server

  
    Serial.print("Connecting to ");
    Serial.println(host);
    if (!client.connect(host, port)) {
        Serial.println("Connection failed.");
        Serial.println("Waiting 5 seconds before retrying...");
        delay(5000);
        return;
    }
    while(client.connected()){
      if (client.available())
      {
        Serial.println("available");
        //read back one line from the server
        String line = client.readString();
        Serial.println(line);
      }
      
      client.write(host);
      client.write('\n');
      client.flush();
      Serial.println("Wrote to server");

      delay(5000);
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
