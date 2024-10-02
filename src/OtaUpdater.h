#ifndef OtaUpdater_h
#define OtaUpdater_h

#include <string>
#include <ESPAsyncWebServer.h>

class OtaUpdater {
    const std::string ssid = "Felix iPhone";
    const std::string password = "1wskwrp0flxv";
    
    AsyncWebServer server;
    bool isActive = false;
    bool isConnected = false;

public:
    OtaUpdater();
    void activate();
    std::string getUpdateUrl();
};

#endif