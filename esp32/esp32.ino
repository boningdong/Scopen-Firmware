/*
 *  This sketch sends a message to a TCP server
 *
 */

#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiAP.h>

const char *ssid = "Scopen";
const char *password = "123456789";
unsigned int port = 2333;

char package_buffer[256];
char reply_buffer[] = "ACK";


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
    const uint16_t port = 6000;
    const char * host = "192.168.4.2"; // ip or dns

    Serial.print("Connecting to ");
    Serial.println(host);

    // Use WiFiClient class to create TCP connections
    WiFiClient client;

    if (!client.connect(host, port)) {
        Serial.println("Connection failed.");
        Serial.println("Waiting 5 seconds before retrying...");
        delay(5000);
        return;
    }

    // This will send a request to the server
    //uncomment this line to send an arbitrary string to the server
    //client.print("Send this data to the server");
    //uncomment this line to send a basic document request to the server
    client.print("GET /index.html HTTP/1.1\n\n");

  int maxloops = 0;

  //wait for the server's reply to become available
  while (!client.available() && maxloops < 1000)
  {
    maxloops++;
    delay(1); //delay 1 msec
  }
  if (client.available() > 0)
  {
    //read back one line from the server
    String line = client.readStringUntil('\r');
    Serial.println(line);
  }
  else
  {
    Serial.println("client.available() timed out ");
  }

    Serial.println("Closing connection.");
    client.stop();

    Serial.println("Waiting 5 seconds before restarting...");
    delay(5000);
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
