#include "Hall.h"

Hall::Hall()
{
}
  
void Hall::begin(int8_t Pin)
{
    HallPin = Pin;
    pinMode(HallPin, INPUT);
}

bool Hall::StateChange(unsigned long SC_Timeout){
    SC_DifTime = millis()-SC_StartTime;
    switch (SC_BestCase)
    {
    case 0:
        SC_StartTime = millis();
        if (!digitalRead(HallPin)){
            SC_BestCase = 1;             //Hall currently high -> waiting for low
        }
        else{
            SC_BestCase = 2;             //Hall currently low -> waiting for high
        }
        break;

    case 1:
        if (digitalRead(HallPin)){
            SC_StartTime = millis();
            SC_BestCase = 2;
            ctStateChange++;
        }
        else if (SC_DifTime >= SC_Timeout)
        {
            SC_BestCase     = 0;
            ctStateChange   = 0;
        }
        break;

    case 2:
        if (!digitalRead(HallPin)){
            SC_StartTime = millis();
            SC_BestCase = 1;
            ctStateChange++;
        }
        else if (SC_DifTime >= SC_Timeout)
        {
            SC_BestCase     = 0;
            ctStateChange   = 0;
        }
        break;

    default:
        break;
    }
    if (ctStateChange >= 3)
    {
        ctStateChange   = 3;
        SC_ReturnVal    = true;
    }else
    {
        SC_ReturnVal    = false;
    }
    
    return SC_ReturnVal;
}