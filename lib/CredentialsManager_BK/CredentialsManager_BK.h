#ifndef CredentialsManager_BK_h
    #define CredentialsManager_BK_h

    #define MaxBytes_BK 320

    #include <Arduino.h>
    #include <ESPAsyncWebServer.h>
    #include "FS.h"
    #include <SPIFFS.h>
    #include <ArduinoJson.h>
    #include <PubSubClient.h>
    
    class CMBK
    {
        public:
            CMBK();
            ~CMBK();
            bool readData();
            StaticJsonDocument<MaxBytes_BK> Credentials;
            void begin();
            //void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len);
        private:
            bool readFile(fs::FS &fs, const char * path);
            void writeFile(fs::FS &fs, const char * path);
            void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
    };
#endif