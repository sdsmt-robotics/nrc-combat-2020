#ifndef __BB_OTA__
#define __BB_OTA__

#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>

bool initAp(const char* ssid, const char* password=NULL);
void initOta();

#endif