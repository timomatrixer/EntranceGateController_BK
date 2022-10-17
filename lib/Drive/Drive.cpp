#include "Drive.h"

Drive::Drive()
{
}
  
void Drive::begin(bool Invert, int8_t ComPin, int8_t LPin)
{
    Drive_ComPin    = ComPin;
    pinMode(Drive_ComPin, OUTPUT); digitalWrite(ComPin, HIGH);
    Drive_LPin      = LPin;
    pinMode(Drive_LPin, OUTPUT);
    InvertDirection = Invert;
    M_WaitTime      = 5000;
}

bool Drive::Move(bool Direction){
    DifTime = millis() - StartTime;
    switch (MoveState)
    {
    case 0://Check actual operation
        MoveReturnVal = false;
        if (DifTime >= M_WaitTime){
            #ifdef Serial_Debug
                Serial.print("D0:");Serial.println(millis());
            #endif
            if (!digitalRead(Drive_ComPin)){                                    //Drive is still on (Inverted Logic)
                if (digitalRead(Drive_LPin) == (InvertDirection ^ Direction)){  //Drive still moves in same direction
                    MoveReturnVal = true;
                }else{                                                          //Direction has to be changed
                    digitalWrite(Drive_ComPin, HIGH);                           //Turning off drive (Inverted Logic)
                    M_WaitTime  = 5000;
                    MoveState   = 1;
                    #ifdef Serial_Debug
                        Serial.println("D1");
                    #endif
                }
            }else{                                                              //Drive is off
                M_WaitTime  = 0;
                MoveState   = 1;
                #ifdef Serial_Debug
                    Serial.println("D2");
                #endif
            }
            StartTime   = millis();                                             //Save time
        }
        break;

    case 1://Set direction
        if (DifTime >= M_WaitTime)
        {
            digitalWrite(Drive_LPin, InvertDirection ^ Direction);              //Set direction
            StartTime   = millis();                                             //Save time
            M_WaitTime  = 500;
            MoveState   = 2;
            #ifdef Serial_Debug
                Serial.print("D3:");Serial.println(millis());
            #endif
        }
        break;
        
    case 2://Turning on Drive
        if (DifTime >= M_WaitTime)
        {
            digitalWrite(Drive_ComPin, LOW);                                    //Turning on drive (Inverted Logic)
            StartTime   = millis();                                             //Save time
            M_WaitTime  = 1000;
            MoveState   = 0;
            MoveReturnVal = true;
            #ifdef Serial_Debug
                Serial.print("D4:");Serial.println(millis());
            #endif
        }
        break;
        
    default:
        break;
    }
    return MoveReturnVal;
}

volatile bool Drive::Stop(){
    if (!digitalRead(Drive_ComPin)){                                            //Drive is still on (Inverted Logic)
        digitalWrite(Drive_ComPin, HIGH);                                       //Turning off drive (Inverted Logic)
        StartTime   = millis();                                                 //Save time
        M_WaitTime  = 5000;
    }
    MoveState   = 0;
    return true;
}