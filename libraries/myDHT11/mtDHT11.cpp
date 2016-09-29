//
// 2015.09.30 Jihan Kim
//

#include "myDHT11.h"


CmyDHT11::CmyDHT11(uint8_t targetPort)
{
    m_TargetPort = targetPort;
    
    // maintain its HIGH status
    pinMode(m_TargetPort, OUTPUT);
    digitalWrite(m_TargetPort, HIGH);
}

CmyDHT11::~CmyDHT11()
{
    ;
}


uint8_t CmyDHT11::checkBit(unsigned long time)
{
    if (time < DECISION_THRESHOLD)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}



unsigned long CmyDHT11::waitFor(uint8_t pinNumber, uint8_t targetStatus, unsigned long maxTime)
{
    unsigned long startCount = micros();
    unsigned long endCount = startCount + maxTime;
    
    while (digitalRead(pinNumber) != targetStatus)
    {
        if (micros() > endCount)
        {
            return -1;
        }
    }
    
    return micros() - startCount;
}



int CmyDHT11::read(float* pTemperature, float* pHumidity)
{
    pinMode(m_TargetPort, OUTPUT);
    digitalWrite(m_TargetPort, LOW);
    delay(18); // Low로 18[ms] 유지해준다.
    digitalWrite(m_TargetPort, HIGH); // HIGH로 복귀
    
    pinMode(m_TargetPort, INPUT);
    
    int t0 = waitFor(m_TargetPort, LOW, 40);    // DHT11이 준비를 기대린다. LOW 까지 최대 40 [us] 기다린다
    if (t0 < 0)
    {
        Serial.println("Error");
        return -1;
    }
    
    int t1 = waitFor(m_TargetPort, HIGH, 90);  // Tx 준비를 체크한다. HIGH까지 최대 90 [us] 까지 기다린다
    if (t1 < 0)
    {
        Serial.println("Error");
        return -1;
    }
    
    int t2 = waitFor(m_TargetPort, LOW, 90);   // Tx 준비를 체크한다. LOW까지 최대 90 [us] 까지 기다린다
    if (t2 < 0)
    {
        Serial.println("Error");
        return -1;
    }
    
    
    // 데이터 전송 시작. 40 bit 읽는다.
    // 습도 정수부 [8bit] + 습도 소수부 [8bit] + 온도 정수부 [8bit] + 온도 소수부 [8bit] + 체크섬 [8bit]
    // HIGH가 유지되는 시간이 짧으면 0, 길면 1이다.
    // 30 [us]를 기준으로 한다.
    int pTimeArray[PACKET_LENGTH];
    for (int idx = 0; idx < PACKET_LENGTH; ++idx)
    {
        if (waitFor(m_TargetPort, HIGH, 50) < 0)    // 데이터 송신 기다린다. HIGH까지 최대 50 [us] 기다린다.
        {
            return -1;
        }
        
        int dataDuration = waitFor(m_TargetPort, LOW, 80);    // 데이터 송신 받는다. 최대 80 [us] 기다린다.
        if (dataDuration < 0)
        {
            return -1;
        }
        
        pTimeArray[idx] = dataDuration;
    }
    
    int RH_Int = 0;
    int RH_Dec = 0;
    int Temp_Int = 0;
    int Temp_Dec = 0;
    int checkSum = 0;
    int bitValue = 0;
    for (int idx = 0; idx < 8; ++idx)
    {
        // 습도 정수부
        RH_Int += (checkBit(pTimeArray[idx]) << (7 - idx));
        
        // 습도 소수부
        RH_Dec += (checkBit(pTimeArray[idx + 8]) << (7 - idx));
        
        
        
        // 온도 정수부
        Temp_Int += (checkBit(pTimeArray[idx + 16]) << (7 - idx));
        
        // 온도 소수부
        Temp_Dec += (checkBit(pTimeArray[idx + 24]) << (7 - idx));
        
        
        // 체크섬
        checkSum += (checkBit(pTimeArray[idx + 32]) << (7 - idx));
    }
    
    
    
    
    if (RH_Int + RH_Dec + Temp_Int + Temp_Dec == checkSum)
    {
        *pTemperature = Temp_Int + Temp_Dec*0.01f;
        *pHumidity = RH_Int + RH_Dec*0.01f;
        return 0;
    }
    else
    {
        return -1;
    }
}