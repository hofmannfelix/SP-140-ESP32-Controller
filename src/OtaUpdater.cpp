#include "OtaUpdater.h"

#include <cstring>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <Adafruit_SleepyDog.h>

OtaUpdater::OtaUpdater() : server(80) {
}

void OtaUpdater::activate() {
    if (isActive) return;
    isActive = true;

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
    Serial.println("");
    auto startConnecting = millis();

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);

        if (millis() - startConnecting < 10000) Watchdog.reset();
    }

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "Update Sp-140 throttle");
    });

    AsyncElegantOTA.begin(&server);
    server.begin();
    isConnected = true;
}

std::string OtaUpdater::getUpdateUrl() {
    if (isConnected)
        return std::string() + WiFi.localIP().toString().c_str() + "/update";
    else
        return "";
}