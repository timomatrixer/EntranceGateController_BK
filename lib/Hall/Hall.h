#ifndef Hall_h
#define Hall_h

#include <Arduino.h>

class Hall
{
    public:
        Hall();
        void begin(int8_t Pin);
        bool StateChange(unsigned long SC_Timeout);
    private:
        byte SC_BestCase;
        unsigned long SC_StartTime;
        long SC_DifTime;
        uint8_t HallPin;
        bool SC_ReturnVal;
        uint8_t ctStateChange;
};
#endif