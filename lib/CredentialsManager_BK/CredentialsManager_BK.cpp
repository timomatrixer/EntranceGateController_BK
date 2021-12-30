#include "CredentialsManager_BK.h"

char data_rx[MaxBytes_BK];
//StaticJsonDocument<MaxBytes_BK> Credentials;

volatile bool NewDataReady = false;

CMBK::CMBK(){}
CMBK::~CMBK(){}

void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
   if(type == WS_EVT_CONNECT){
      #ifdef Serial_Debug 
         Serial.println("Websocket client connection received");
         Serial.println("------------------------------------");
      #endif
   } else if(type == WS_EVT_DISCONNECT){
      #ifdef Serial_Debug 
         Serial.println("Client disconnected");
         Serial.println("------------------------------------");
      #endif
 
   } else if(type == WS_EVT_DATA){
      memset(data_rx, 0, sizeof(data_rx));
      for (size_t i = 0; i < len; i++)
      {
         data_rx[i] = data[i];
         #ifdef Serial_Debug 
            Serial.print(data_rx[i]);
         #endif
      }

      NewDataReady = true;
      /*
      DeserializationError error = deserializeJson(Credentials, data);
      if (error){
         #ifdef Serial_Debug 
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
         #endif
         return;
      } else{
         #ifdef Serial_Debug 
            Serial.print("Server data: Length: ");Serial.println(len);
            serializeJsonPretty(Credentials, Serial);
            Serial.println();
         #endif
         NewDataReady = true;
      }
      */
   }
}

bool CMBK::readData(){
   SPIFFS.begin();   
   //listDir(SPIFFS, "/", 0);
   if (readFile(SPIFFS, "/para.txt")){
      return true;
   }else{
      return false;
   }
}

void CMBK::begin(){
   SPIFFS.begin();

   AsyncWebServer BKserver(80);
   AsyncWebSocket BKws("/paraex");
   WiFiClient     BKWiFi_client;
   PubSubClient   BKMQTT_client(BKWiFi_client);

   WiFi.disconnect(false,false);
   WiFi.mode(WIFI_AP_STA);

   char ApSsid[8] = "CMBK";
   uint8_t NumberOfCMBK = 0;
   int NumberOfSSID     = WiFi.scanNetworks();
   if (NumberOfSSID != 0) {
      for (int i = 0; i < NumberOfSSID; ++i) {
         // Print SSID and RSSI for each network found
         if (WiFi.SSID(i) == "CMBK"){
            NumberOfCMBK++;
         }
      }
      char cNumberOfCMBK[4];
      itoa(NumberOfCMBK, cNumberOfCMBK, 10);
      strcat(ApSsid,cNumberOfCMBK);
   }
   #ifdef Serial_Debug
      Serial.print("AP name: ");Serial.println(ApSsid);
   #endif

   WiFi.softAP(ApSsid);

   #ifdef Serial_Debug
      IPAddress IP = WiFi.softAPIP();
      Serial.print("IP address: ");Serial.println(IP);
   #endif

   BKws.onEvent(onWsEvent);
   BKserver.addHandler(&BKws);
   BKserver.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/CMBK.html", "text/html");
   });

   BKserver.begin();
   delay(1000);

   bool WifiCredentialsOk = false;
   bool MQTTCredentialsOk = false;

   while (!WifiCredentialsOk || !MQTTCredentialsOk){
      if (NewDataReady) {
         NewDataReady      = false;
         WifiCredentialsOk = false;
         MQTTCredentialsOk = false;

         // Get new data
         deserializeJson(Credentials,data_rx);
         const char* ctrlnametry    = Credentials["NAME"];
         const char* ssidtry        = Credentials["WIFI_SSID"];
         const char* pwtry          = Credentials["WIFI_PW"];
         const char* mqttiptry      = Credentials["MQTT_IP"];
         const uint16_t mqttporttry = Credentials["MQTT_PORT"];
         const char* mqttidtry      = Credentials["MQTT_ID"];
         const char* mqttpwtry      = Credentials["MQTT_PW"];
         // Configure and start the WiFi test
         WiFi.begin     (ssidtry, pwtry);

         #ifdef Serial_Debug
            uint8_t WifiStatusOld   = 0;
         #endif
         unsigned long StartTime = millis();
         bool WifiTest           = true;
         BKws.textAll("Start Wifi test");
         // Wifi connection test
         while (WifiTest) {
            #ifdef Serial_Debug
               if (WifiStatusOld != WiFi.status()) {
                  WifiStatusOld = WiFi.status();
                  Serial.print("WiFiStatus: ");Serial.println(WifiStatusOld);
               }
            #endif
            
            if (WiFi.status() == WL_CONNECTED) {
               #ifdef Serial_Debug
                  Serial.println("Wifi credentials ok!");
               #endif
               WifiCredentialsOk = true;
               WifiTest          = false;                      //exit while
            }else if ((millis()-StartTime)>5000)
            {
               if ((WiFi.status() == WL_NO_SSID_AVAIL)
               ||  (WiFi.status() == WL_CONNECT_FAILED)) {
                  #ifdef Serial_Debug
                     Serial.println("Wrong wifi credentials!");
                  #endif
                  BKws.textAll("wifi");                             //Wrong wifi Credentials
                  WifiTest          = false;                      //exit while
               }else if ((millis()-StartTime) > 15000) {
                  #ifdef Serial_Debug
                     Serial.println("Connection timeout!");
                  #endif
                  BKws.textAll("to");                               //timeout
                  WifiTest          = false;                      //exit while
               }
            }
         }
         // MQTT test
         if (WifiCredentialsOk) {
            #ifdef Serial_Debug
               Serial.println("Start mqtt test");
            #endif
            
            BKMQTT_client.disconnect();
            BKMQTT_client.setServer("", 0);
            BKMQTT_client.setServer(mqttiptry, mqttporttry);

            uint8_t retries   = 1;
            bool MQTTTest     = true;
            BKws.textAll("Start MQTT test");
            while (!BKMQTT_client.connected() && MQTTTest) {
               // Attempt to connect
               if (BKMQTT_client.connect(ctrlnametry,mqttidtry,mqttpwtry,0,1,0,0,1)) {
                  #ifdef Serial_Debug
                     Serial.println("connected");
                  #endif
                  MQTTCredentialsOk = true;
                  MQTTTest          = false;
               } else {
                  #ifdef Serial_Debug
                     Serial.print("failed, rc=");
                     Serial.print(BKMQTT_client.state());
                  #endif
                  BKws.textAll("Retrying MQTT connection..." + (String)retries);
                  retries++;
               }

               if (retries > 3)
               {
                  #ifdef Serial_Debug
                     Serial.println("Can not connect to mqtt broker!");
                  #endif
                  BKws.textAll("mqtt");           //Wrong mqtt Credentials
                  MQTTTest          = false;
               }
            }
         }
      }
   }
   BKws.textAll("ok");           //Credentials ok

   #ifdef Serial_Debug
      Serial.println("Credentials ok!");
   #endif

   writeFile(SPIFFS,"/para.txt");
   delay(5000);
   ESP.restart();
}

bool CMBK::readFile(fs::FS &fs, const char * path){
   File file = fs.open(path);
   if(!file || file.isDirectory()){
      #ifdef Serial_Debug 
         Serial.println("Failed to open file for reading");
      #endif
      return false;
   }

   DeserializationError error = deserializeJson(Credentials, file);
   if (error){
      #ifdef Serial_Debug 
         Serial.println(F("Failed to read file, using default configuration"));
      #endif
      file.close();
      return false;
   }else {
      #ifdef Serial_Debug 
         Serial.println("Read from file:");
         serializeJsonPretty(Credentials, Serial); 
      #endif
      file.close();
      return true;
   }
}

void CMBK::writeFile(fs::FS &fs, const char * path){
   File file = fs.open(path, FILE_WRITE);
   if(!file){
      #ifdef Serial_Debug 
         Serial.println("Failed to open file for writing");
      #endif
      return;
   }
   if (serializeJson(Credentials, file) == 0) {
      #ifdef Serial_Debug 
         Serial.println(F("Failed to write to file"));
      #endif
   }else {
      #ifdef Serial_Debug 
         Serial.println("Write File done");
      #endif
   }
   file.close();
}

void CMBK::listDir(fs::FS &fs, const char * dirname, uint8_t levels){
   File root = fs.open(dirname);
   if(!root){
      #ifdef Serial_Debug 
         Serial.println("Failed to open directory");
      #endif
      return;
   }
   if(!root.isDirectory()){
      #ifdef Serial_Debug 
         Serial.println(" not a directory");
      #endif
      return;
   }

   File file = root.openNextFile();
   while(file){
      if(file.isDirectory()){
         #ifdef Serial_Debug 
            Serial.print("  DIR : ");
            Serial.println(file.name());
         #endif
         if(levels){
            listDir(fs, file.name(), levels -1);
         }
      } else {
         #ifdef Serial_Debug 
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
         #endif
      }
      file = root.openNextFile();
   }
   file.close();
}