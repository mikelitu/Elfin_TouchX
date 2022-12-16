#pragma once

#ifndef UINT
typedef unsigned int UINT;
#endif

#ifndef UCHAR
typedef unsigned char UCHAR;
#endif

/**
* @brief: CLinuxSerial class
*/
class CLinuxSerial
{
public:
    float sensor[6];
    unsigned char CheckSum(unsigned char *buf, const int len);
    CLinuxSerial();
    CLinuxSerial(UINT portNo = 0 , UINT baudRate = 115200 );
    ~CLinuxSerial();
    void Sensor();

    bool InitPort(UINT portNo = 0, UINT baudRate = 115200);
    UINT ReadData(UCHAR *data, UINT length);
    UINT WriteData(UCHAR *data, UINT length);
    UINT GetBytesInCom();
    //void ReadSensor();

private:
    int m_iSerialID;
    bool OpenPort(UINT portNo);
    void ClosePort();
};