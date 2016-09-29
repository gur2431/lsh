
// myDHT11.h
// 2015.09.30 Jihan Kim

#ifndef _MYDHT11_h
#define _MYDHT11_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

const int PACKET_LENGTH = 40;    // [bits]
const int DECISION_THRESHOLD = 30;    // [us]

class CmyDHT11
{
public:
    CmyDHT11(uint8_t targetPortNum);
    ~CmyDHT11();
    
    
    int read(float* pTemperature, float* pHumidity);
private:
    
    unsigned long waitFor(uint8_t pinNumber, uint8_t targetStatus, unsigned long maxTime);
    uint8_t checkBit(unsigned long time);
    uint8_t m_TargetPort;
};


#endif