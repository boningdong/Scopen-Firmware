#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>
#include <Arduino.h>

const char *ssid = "Scopen";
const char *password = "123456789";
unsigned int port = 2333;

char package_buffer[256];
char reply_buffer[] = "ACK";

WiFiUDP udp;

void setup()
{
  Serial.begin(115200);
  Serial.println();

  Serial.println("Creating AP ...");
  WiFi.softAP(ssid, password);

  IPAddress ip = WiFi.softAPIP();
  Serial.println("AP IP address: ");
  Serial.println(ip);
  WiFi.onEvent(wifi_event_handler);

  udp.begin(port);
}

void loop()
{
  // printf("...");
  int packsize = udp.parsePacket();
  if (packsize != 0) {
    IPAddress ip = udp.remoteIP();
    Serial.print("Received packets from remote ip: ");
    Serial.print(ip);
    Serial.print(" port: ");
    Serial.println(udp.remotePort());
    udp.read(package_buffer, 256);
    Serial.println(package_buffer);
    
    const uint8_t msg[] = "Got it!\n";
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write(msg, strlen((const char*)msg));
    udp.endPacket();
    
  } else {
    IPAddress ip = WiFi.localIP();
    ip[3] = 255;

    const uint8_t info[] = "conncet scopen";
//    String s = String(local_ip);
//    const char strip[s.length()];
//    for (int i = 0; i < s.length(); i++) {
//      strip[i] = s[i];
//    }
    
    udp.beginPacket(ip, port);
    udp.write(info, strlen((const char*)info));
//    udp.write(strip, strlen(strip));
    udp.endPacket();  
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
