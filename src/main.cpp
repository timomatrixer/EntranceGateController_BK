/***************************************************
  Must use EntranceGateController BK from:
    https://

  Written by Timo Buntrock. Only Open Source
 ****************************************************/
#define ESP32_RTOS

#include <Arduino.h>
#include "CredentialsManager_BK.h"
#include "Hall.h"
#include "Drive.h"
#include "OTA.h"
#include <TelnetStream.h>

/************************* Controller Setup *********************************/
#define PProbe_In         14    //Photo Probe pin (obstacle detection)
#define IProbe_In         15    //Inductive Probe pin (endstops)
#define Main_StartTime    3500
#define Main_StopTime     5000

CMBK  Manager;
Hall  EGC_Hall;
Drive EGC_Drive;

/************ Global State (you don't need to change this!) ******************/

// Create an ESP WiFiClient class to connect to the MQTT server.
WiFiClient WiFi_client;
// or... use WiFiClientSecure for SSL
//WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
PubSubClient MQTT_client(WiFi_client);

/****************************** Feeds ***************************************/
#define CmdTopic "/Cmd"
#define StateTopic "/State"
char MqttCmdUrl[20];
char MqttStateUrl[20];

char MqttData[4];
bool NewCmd;                                            //New Command from broker

void callback(char* topic, byte* payload, unsigned int length) {
  memset(MqttData,0,sizeof(MqttData));
  for (int i=0;i<min(length, sizeof(MqttData));i++) {
    MqttData[i] = (char)payload[i];
    #ifdef TelnetStream_Debug
      TelnetStream.print(MqttData[i]);
    #endif
  }
  #ifdef TelnetStream_Debug
    TelnetStream.print(" <- Message arrived from ");
    TelnetStream.println(topic);
  #endif
  NewCmd  = true;
}

enum {  State_None = 0,     //0
        State_Close,        //1
        State_Open,         //2
        State_Stop,         //3
        State_Closing,      //4
        State_Opening,      //5
        State_Obstacle,     //6
        State_Jammed,       //7
        State_ProbeProblem, //8
        State_AnyPosition,  //9
        State_Reset         //10
      };
unsigned long MQTT_AliveTimer;

/*************************** Sketch Code ************************************/

volatile uint8_t Statemachine;
char MQTT_CmdData;
int32_t MQTT_State;
int32_t MQTT_StateOld;
int32_t Position_State = State_Close;
uint8_t Command;
uint8_t CommandOld;
uint8_t ctObstacle;                                     //Counter for Photo_Probe filtering
bool FirstRun = true;                                   //Controller just started and will get first cmd
bool Moving;                                            //Status Drive is moving
bool blink;
uint8_t ctAliveOnTelnet;
unsigned long     Main_SaveTime;
long              Main_DifTime;
unsigned long     Main_Time;
byte              Main_Hour;
byte              Main_Minute;
byte              Main_Second;

void CheckCredentials();
void MQTT_setup();
void MQTT_connect();
void Wifi_Reconnect();
void Telnet_Input();

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  #ifdef Serial_Debug
    Serial.println("Disconnected from WiFi access point");
    Serial.print("WiFi lost connection. Reason: ");
    Serial.println(info.disconnected.reason);
    Serial.println("Trying to Reconnect");
  #endif
  WiFi.disconnect(true);
  const char* ssid_event  = Manager.Credentials["WIFI_SSID"];
  const char* pw_event    = Manager.Credentials["WIFI_PW"];
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid_event, pw_event);
}

void IRAM_ATTR PProbe_ISR() { 
  detachInterrupt(PProbe_In);                           //Denied interrupt of Photoprobe
  EGC_Drive.Stop();
  Statemachine = 90;
};

void setup() {
  #ifdef Serial_Debug
    Serial.begin(4800);//115200);
  #endif

  CheckCredentials();

  setupOTA( Manager.Credentials["NAME"]);

  pinMode(PProbe_In,INPUT_PULLUP);                      //Photo Probe
  pinMode(IProbe_In,INPUT);                             //Inductive Probe

  EGC_Hall.begin(5);
  EGC_Drive.begin(false, 17, 16);

  pinMode(3,OUTPUT); 
}

void loop() {
  Telnet_Input();

  MQTT_connect();
  
  MQTT_client.loop();

  if (NewCmd) {
    MQTT_CmdData = atoi(MqttData);
    if (MQTT_CmdData == State_Open) {                                //Command opening gate
      Command = State_Open;
      #ifdef TelnetStream_Debug
        TelnetStream.println("Open");
      #endif
    }else if (MQTT_CmdData == State_Close) {                          //Command closing gate
      Command = State_Close;
      #ifdef TelnetStream_Debug
        TelnetStream.println("Close");
      #endif
    }else if (MQTT_CmdData == State_Stop) {                          //Fast Stop
      Command = State_Stop;
      //detachInterrupt(PProbe_In);                               //Denied interrupt of Photoprobe
      EGC_Drive.Stop();
      #ifdef TelnetStream_Debug
        TelnetStream.println("Stop");
      #endif
    }else if (MQTT_CmdData == State_Reset) {                          //Reset
      Command = State_None;
      //detachInterrupt(PProbe_In);                               //Denied interrupt of Photoprobe
      EGC_Drive.Stop();
      Position_State = State_None;
      #ifdef TelnetStream_Debug
        TelnetStream.println("Reset");
      #endif
    }else {
      NewCmd  = false;
    }
    
    if (NewCmd) {
      NewCmd  = false;
      if (FirstRun) {                                           //Controller just started
        CommandOld    = Command;
        #ifdef TelnetStream_Debug
          TelnetStream.print("M0:");TelnetStream.println(Main_DifTime);
        #endif
        //detachInterrupt(PProbe_In);                             //Denied interrupt of Photoprobe
        EGC_Drive.Stop();                                       //First start first stop
        if (Main_DifTime >10000UL) {                              //After 10s first cmd allowed
          CommandOld    = 0;
          Main_SaveTime = millis();
          FirstRun      = false;
          #ifdef TelnetStream_Debug
            TelnetStream.print("M15:");TelnetStream.println(millis());
          #endif
        }
      }else if((Statemachine > 1) && (CommandOld != Command)) { //Drive is still moveing and has to be stopped first
        Statemachine = 1;
        //detachInterrupt(PProbe_In);                             //Denied interrupt of Photoprobe
        EGC_Drive.Stop();
      }
    }
  }
  
  Moving = /*true;*/EGC_Hall.StateChange(Main_StartTime);
  if (Moving) {
    if (!digitalRead(IProbe_In)) {
      Position_State  = State_AnyPosition;            //Gate position is anywhere between endstops
    }
    //if (digitalRead(PProbe_In)) {
    //  EGC_Drive.Stop();
    //  Statemachine = 90;                               //Obstacle
    //}
  }

  switch (Statemachine) {
  case 0://First Start or drive was jammed
    if (!FirstRun) {
      #ifdef TelnetStream_Debug
        TelnetStream.print("M1:");TelnetStream.println(Main_Time);
      #endif
      Statemachine = 1;
    }
    break;

  case 1://Waiting
    if (CommandOld != Command) {
      CommandOld = Command;
      if ((Command == State_Open)&&(Position_State != State_Open)) {
        MQTT_State    = State_Opening;
        Statemachine  = 2;
        #ifdef TelnetStream_Debug
          TelnetStream.print("M2:");TelnetStream.println(Main_Time);
        #endif
      }else if ((Command == State_Close)&&(Position_State != State_Close)) {
        MQTT_State    = State_Closing;
        Statemachine  = 3;
        #ifdef TelnetStream_Debug
          TelnetStream.print("M3:");TelnetStream.println(Main_Time);
        #endif
      }else if (Command == State_Stop) {
        MQTT_State    = State_Stop;
        #ifdef TelnetStream_Debug
          TelnetStream.print("M15:");TelnetStream.println(Main_Time);
        #endif
      }
    }
    break;

  case 2://Opening
    if (EGC_Drive.Move(Drive_Open)) {
      if (digitalRead(IProbe_In)) {
        #ifdef TelnetStream_Debug
          TelnetStream.print("M4:");TelnetStream.println(Main_Time);
        #endif
        Statemachine = 10;  //Drive stands on endstop -> Waiting for moveing and leaving endstop
      }else {
        #ifdef TelnetStream_Debug
          TelnetStream.print("M5:");TelnetStream.println(Main_Time);
        #endif
        Statemachine = 20;  ///Drive stands anywhere -> Waiting for moveing
      }
      Main_SaveTime  = Main_Time;
    }
    break;

  case 3://Closing
    if (EGC_Drive.Move(Drive_Close)) {
      if (digitalRead(IProbe_In)) {
        #ifdef TelnetStream_Debug
          TelnetStream.print("M6:");TelnetStream.println(Main_Time);
        #endif
        Statemachine = 10;  //Drive stands on endstop -> Waiting for moveing and leaving endstop
      }else {
        #ifdef TelnetStream_Debug
          TelnetStream.print("M7:");TelnetStream.println(Main_Time);
        #endif
        Statemachine = 20;  ///Drive stands anywhere -> Waiting for moveing
      }
      Main_SaveTime  = Main_Time;
    }
    break;

  case 10://Start from Endstop
    //attachInterrupt(PProbe_In,PProbe_ISR, RISING);  //Allow interrupt of Photoprobe
    if (Moving && !digitalRead(IProbe_In)) {
      #ifdef TelnetStream_Debug
        TelnetStream.print("M8:");TelnetStream.println(Main_Time);
      #endif
      Statemachine = 30;
    }else if (Main_DifTime > Main_StartTime) {
      if (digitalRead(IProbe_In) && Moving) {
        MQTT_State    = State_ProbeProblem; //Drive is moveing but inductive probe is still On
      }else {
        MQTT_State    = State_Jammed;       //Drive did not move
      }
      #ifdef TelnetStream_Debug
        TelnetStream.print("M9:");TelnetStream.println(Main_Time);
      #endif
      Statemachine  = 1;
      //detachInterrupt(PProbe_In);                       //Denied interrupt of Photoprobe
      EGC_Drive.Stop();
    }
    break;

  case 20://Start from anywhere
    //attachInterrupt(PProbe_In,PProbe_ISR, RISING);  //Allow interrupt of Photoprobe
    if (Moving) {
      #ifdef TelnetStream_Debug
        TelnetStream.print("M10:");TelnetStream.println(Main_Time);
      #endif
      Statemachine = 30;
    }else if (Main_DifTime > Main_StartTime) {
      MQTT_State    = State_Jammed;         //Drive did not move
      #ifdef TelnetStream_Debug
        TelnetStream.print("M11:");TelnetStream.println(Main_Time);
      #endif
      Statemachine  = 1;
      //detachInterrupt(PProbe_In);                       //Denied interrupt of Photoprobe
      EGC_Drive.Stop();
    }
    break;

  case 30://Waiting for endstop
    if (digitalRead(IProbe_In)) {
      //detachInterrupt(PProbe_In);                       //Denied interrupt of Photoprobe
      EGC_Drive.Stop();
      if (Command == State_Open) {
        MQTT_State      = State_Open;       //Gate open
        Position_State  = State_Open;       //Save state: Gate open
      }else if (Command == State_Close) {
        MQTT_State      = State_Close;      //Gate close
        Position_State  = State_Close;      //Save state: Gate close
      }
      #ifdef TelnetStream_Debug
        TelnetStream.print("M12:");TelnetStream.println(Main_Time);
      #endif
      Statemachine  = 1;
    }
    break;

  case 90://Obstacle detected
    MQTT_State    = State_Obstacle;
    if (!digitalRead(PProbe_In)) {
      ctObstacle++;
      if (ctObstacle > 100) {                                         //Simple Photo_Probe filtering 
        ctObstacle = 0;
        if (Position_State == State_AnyPosition) {
          CommandOld = 0;                                             //Obstacle away -> Start again
        }
        #ifdef TelnetStream_Debug
          TelnetStream.print("M13:");TelnetStream.println(Main_Time);
        #endif
        if (FirstRun) {
          Statemachine  = 0;
        }else {
          Statemachine  = 1;
        }
      }
    }else {
      ctObstacle = 0;
    }
    break;

  default:
    break;
  }

  if ((Main_Time - MQTT_AliveTimer > 999UL)
  /*|| ((Main_Time - MQTT_AliveTimer > 999UL) && (MQTT_State != MQTT_StateOld))*/){                 //delay of 1s reached
    blink         = !blink;
    digitalWrite(3,blink);
    MQTT_AliveTimer = Main_Time;
    //if (MQTT_State != MQTT_StateOld){         //New State 
      MQTT_StateOld = MQTT_State;
      char payloadPublish[4];
      itoa(MQTT_State, payloadPublish, 10);
      if (MQTT_client.publish(MqttStateUrl,payloadPublish)){
        ctAliveOnTelnet++;
        if (ctAliveOnTelnet >= 60){             //Every minute send Alive over Telnet
          ctAliveOnTelnet = 0;
          #ifdef TelnetStream_Debug
            TelnetStream.print(payloadPublish);TelnetStream.print(" on: ");TelnetStream.print(MqttStateUrl);TelnetStream.println(Main_Time);
          #endif
        }
      }else {
        #ifdef TelnetStream_Debug
          TelnetStream.println("Publish fail!");
        #endif
      }
    //}
  }
}

void CheckCredentials() {
  //if (touchRead(12) < 40) {
  //  #ifdef Serial_Debug
  //      Serial.println("TouchPad while uC start! Start Server");
  //  #endif
  //  Manager.begin();
  //}
  
  if (Manager.readData()) {
    bool StartServer        = false;

    const char* ctrlname    = Manager.Credentials["NAME"];
    const char* ssid        = Manager.Credentials["WIFI_SSID"];
    const char* pw          = Manager.Credentials["WIFI_PW"];
    const char* mqttid      = Manager.Credentials["MQTT_ID"];
    const char* mqttpw      = Manager.Credentials["MQTT_PW"];
    
    WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED/*ARDUINO_EVENT_WIFI_STA_DISCONNECTED*/);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pw);


    unsigned long CC_StartTime = millis();
    unsigned long CC_Time = millis();

    while ((WiFi.status() != WL_CONNECTED) && !StartServer) {         //Attempt to connect with Wifi
      if ((WiFi.status() == WL_NO_SSID_AVAIL)
      ||  (WiFi.status() == WL_CONNECT_FAILED)) {
        #ifdef Serial_Debug
            Serial.println("Wrong Credentials! Start Server");
        #endif
        StartServer = true;
      }
      CC_Time = millis();
      if ((CC_Time-CC_StartTime) > 240000UL)                               //After 240s -> Start Server
      {
        #ifdef Serial_Debug
            Serial.println("Connection Failed! Start Server");
        #endif
        //ESP.restart();
        StartServer = true;
      }
    }

    TelnetStream.begin(23);

    if (!StartServer) {
      MQTT_setup();

      uint8_t retries   = 1;
      while (!MQTT_client.connected() && !StartServer) {
        if (MQTT_client.connect(ctrlname,mqttid,mqttpw,0,1,0,0,1)) {  //Attempt to connect
          #ifdef TelnetStream_Debug
            TelnetStream.println("connected");
          #endif
          //Subscribe: Cmd Topic
          strcpy(MqttCmdUrl, ctrlname);
          strcat(MqttCmdUrl, CmdTopic);
          MQTT_client.subscribe(MqttCmdUrl);
          //Publish: State Topic
          strcpy(MqttStateUrl, ctrlname);
          strcat(MqttStateUrl, StateTopic);
          char payloadPublish[4];
          itoa(State_None, payloadPublish, 10);
          MQTT_client.publish(MqttStateUrl,payloadPublish);
        } else {                                                      //Fail
          #ifdef TelnetStream_Debug
            TelnetStream.print("failed, rc=");
            TelnetStream.print(MQTT_client.state());
          #endif
          retries++;
        }
        if (retries > 3) {                                            //After 3 attempts -> Start Server
          #ifdef TelnetStream_Debug
            TelnetStream.println("Can not connect to mqtt broker!");
          #endif
          //ESP.restart();
          StartServer          = true;
        }
      }
    }
    if (StartServer)
    {
      Manager.begin();
    }
  }
}

void MQTT_setup(){
  const char* mqttip      = Manager.Credentials["MQTT_IP"];
  const uint16_t mqttport = Manager.Credentials["MQTT_PORT"];
  MQTT_client.setServer(mqttip, mqttport);
  MQTT_client.setKeepAlive(43200);//alle 12h
  MQTT_client.setCallback(callback);
}

// Function to connect and reconnect as necessary to the MQTT server
// Is called in the loop function and it will take care if connecting.
void MQTT_connect() {
  if (MQTT_client.state() == MQTT_CONNECTED) {             //Return if already connected.
    return;
  }

  #ifdef TelnetStream_Debug
    TelnetStream.print("Connecting to MQTT... ");
  #endif

  MQTT_setup();
  uint8_t retries   = 1;
  while (!MQTT_client.connected()) {
    // Attempt to connect
    if (MQTT_client.connect(Manager.Credentials["NAME"],Manager.Credentials["MQTT_ID"],Manager.Credentials["MQTT_PW"],0,1,0,0,1)) {
        //Subscribe: Cmd Topic
        MQTT_client.subscribe(MqttCmdUrl);
        //Publish: State Topic
        char payloadPublish[4];
        itoa(Command, payloadPublish, 10);
        MQTT_client.publish(MqttStateUrl,payloadPublish);
      #ifdef TelnetStream_Debug
        TelnetStream.println("connected");
      #endif
    } else {
      #ifdef TelnetStream_Debug
        TelnetStream.print("failed, rc=");
        TelnetStream.print(MQTT_client.state());
      #endif
      retries++;
    }

    if (retries > 3)
    {
      #ifdef TelnetStream_Debug
        TelnetStream.println("Can not connect to mqtt broker!");
      #endif
      if (WiFi.status() != WL_CONNECTED) {
        return;
      }
    }
  }
  
  Statemachine  = 0;
  FirstRun      = true;
  Main_SaveTime = millis();
  Main_DifTime  = 0;
  CommandOld    = Command;
  #ifdef TelnetStream_Debug
    TelnetStream.println("MQTT Connected!");
  #endif
}

void Telnet_Input(){
  Main_Time = millis();
  Main_DifTime = Main_Time - Main_SaveTime;
  unsigned long Main_millis = Main_Time;
  Main_Hour = (Main_millis / 3600000);
  Main_millis -= (Main_Hour * 3600000);
  Main_Minute = (Main_millis / 60000);
  Main_millis -= (Main_Minute * 60000);
  Main_Second = (Main_millis / 1000);

/*Telnet input*/
  switch (TelnetStream.read()) {
    case 'i':
      #ifdef TelnetStream_Debug
        //const char* TelnetNameForShow    = Manager.Credentials["NAME"];
        //TelnetStream.println(TelnetNameForShow);
        TelnetStream.print("Time             : ");TelnetStream.print(Main_Hour);TelnetStream.print("h");TelnetStream.print(Main_Minute);TelnetStream.print("m");TelnetStream.print(Main_Second);TelnetStream.println("s");
        TelnetStream.print("Last Cmd         : ");
        switch (CommandOld){
          case 0:   TelnetStream.println("None"); break;
          case 1:   TelnetStream.println("Close"); break;
          case 2:   TelnetStream.println("Open"); break;
          case 3:   TelnetStream.println("Stop"); break;
          case 10:  TelnetStream.println("Reset"); break;
          default:  TelnetStream.println(CommandOld); break;
        }
        TelnetStream.print("State            : ");
        switch (MQTT_State){
          case 0:   TelnetStream.println("None"); break;
          case 1:   TelnetStream.println("Close"); break;
          case 2:   TelnetStream.println("Open"); break;
          case 3:   TelnetStream.println("Stop"); break;
          case 4:   TelnetStream.println("Closing"); break;
          case 5:   TelnetStream.println("Opening"); break;
          case 6:   TelnetStream.println("Obstacle"); break;
          case 7:   TelnetStream.println("Jammed"); break;
          case 8:   TelnetStream.println("ProbeProblem"); break;
          case 9:   TelnetStream.println("AnyPosition"); break;
          case 10:  TelnetStream.println("Reset"); break;
          default:  TelnetStream.println(MQTT_State); break;
        }
        TelnetStream.print("Induc Probe State: ");TelnetStream.println(digitalRead(IProbe_In));
        TelnetStream.print("Photo Probe State: ");TelnetStream.println(digitalRead(PProbe_In));
        TelnetStream.print("Relay Com State  : ");TelnetStream.println(digitalRead(17));
        TelnetStream.print("Relay L State    : ");TelnetStream.println(digitalRead(16));
      #endif
      break;
    case 'R':
      TelnetStream.stop();
      delay(100);
      ESP.restart();
      break;
      
    default:
    break;
  }
}