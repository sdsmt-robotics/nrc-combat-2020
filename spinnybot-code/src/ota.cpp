#include "ota.h"

IPAddress local_IP(10, 1, 1, 1);
IPAddress gateway(10, 1, 1, 0);
IPAddress subnet(255, 255, 255, 0);

// Task running on other core (core 0)
TaskHandle_t otaTask;

/**
 * OTA update hander running on other core.
 */
void otaHandler(void * pvParameters) {
    while (true){
        delay(1);  // Delays feed the watchdog timer (WDT). Otherwise we crash.
        ArduinoOTA.handle();
    }
}

/**
 * Initialize the access point.
 * 
 * @return true if successful, false otherwise. 
 */
bool initAp(const char* ssid, const char* password) {
    bool success = WiFi.softAPConfig(local_IP, gateway, subnet);
    if (!success) {
        return false;
    }
    success = WiFi.softAP(ssid, password);

    return success;
}

/**
 * Initialize the OTA updater.
 */
void initOta() {
    ArduinoOTA
        .onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
                type = "sketch";
            else  // U_SPIFFS
                type = "filesystem";

            // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
            Serial.println("Start updating " + type);
        })
        .onEnd([]() {
            Serial.println("\nEnd");
        })
        .onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR)
                Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR)
                Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR)
                Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR)
                Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR)
                Serial.println("End Failed");
        });

    ArduinoOTA.begin();
    xTaskCreatePinnedToCore(
                    otaHandler,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &otaTask,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
}