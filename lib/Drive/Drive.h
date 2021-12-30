#ifndef Drive_h
#define Drive_h

#include <Arduino.h>

#define Drive_Open          false
#define Drive_Close         true

class Drive
{
    public:
        Drive();
        void begin(bool Invert, int8_t ComPin, int8_t LPin);
        volatile bool Stop();
        bool Move(bool Direction);
    private:
        int8_t Drive_ComPin;
        int8_t Drive_LPin;
        uint8_t MoveState;
        unsigned long StartTime;
        long DifTime;
        bool InvertDirection;
        bool MoveReturnVal;
        unsigned long M_WaitTime;
};
#endif