#pragma once
#ifndef __CANManager_h
#define __CANManager_h

#include <MCP_CAN.h>

#define CAN0_DEFAULT_INT A1 // Set INT to pin A1
#define CAN0_DEFAULT_CS A2  // Set CS to pin A2
#define CAN_DATA_BUFFER_SIZE 8

class CANManager
{
  public:
    CANManager(int IntPin, int CSPin, bool debugOn = false);
    ~CANManager();

    void update();
    void log(String str, bool blnWithNewLine = false);

    void setDebug(bool blnDebug);
    bool getDebug();

    void setCANDataReady(bool blnCANDataReady);
    bool isCANDataReady();

    void setCANData(unsigned char *data);
    unsigned char *getCANData();

  private:
    int iIntPin = CAN0_DEFAULT_INT;
    int iCSPin = CAN0_DEFAULT_CS;
    bool blnDebugOn = false;
    bool blnCANDataReady = false;
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char data[CAN_DATA_BUFFER_SIZE];

    MCP_CAN CAN0;
};

#endif // def(__CANManager_h)
