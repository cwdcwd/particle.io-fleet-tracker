#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>
#include "CANManager.h"

CANManager::CANManager(int IntPin, int CSPin, bool debugOn) : iIntPin(IntPin), iCSPin(CSPin), blnDebugOn(debugOn), CAN0(iCSPin) {
  pinMode(iIntPin, INPUT);  // Configuring pin for /INT input
  // CAN0 = MCP_CAN(iCSPin);   // Set CS for CAN0
  CAN0.setMode(MCP_NORMAL); // Set operation mode to normal so the MCP2515 sends acks to received data.

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
  {
    if (blnDebugOn)
    {
      log("MCP2515 Initialized Successfully!");
    }
  }
  else
  {
    if (blnDebugOn)
    {
      log("Error Initializing MCP2515...");
    }
  }
}

CANManager::~CANManager() {
    // Nothing to do here
}

void CANManager::update() {
  if (!digitalRead(iIntPin)) // If iIntPin pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, data); // Read data: len = data length, buf = data byte(s)

    if(blnDebugOn) {
      char msgString[128];
      if ((rxId & 0x80000000) == 0x80000000) // Determine if ID is standard (11 bits) or extended (29 bits)
        sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
      else
        sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);

      log(msgString);

      if ((rxId & 0x40000000) == 0x40000000)
      { // Determine if message is a remote request frame.
        sprintf(msgString, " REMOTE REQUEST FRAME");
        log(msgString);
      }
      else
      {
        for (byte i = 0; i < len; i++)
        {
          sprintf(msgString, " 0x%.2X", data[i]);
          log(msgString, false);
        }
      }
    }

    blnCANDataReady = true;
  }
}

void CANManager::log(String str, bool blnWithNewLine) {
  if (blnDebugOn) {
    if (blnWithNewLine) {
      Serial.println(str);
    } else {
      Serial.print(str);
    }
  }
}

void CANManager::setDebug(bool blnDebug) {
    blnDebugOn = blnDebug;
}

bool CANManager::getDebug() {
    return blnDebugOn;
}

bool CANManager::isCANDataReady() {
    return blnCANDataReady;
}

void CANManager::setCANDataReady(bool blnCANDataReady) {
    this->blnCANDataReady = blnCANDataReady;
}

void CANManager::setCANData(unsigned char *data) {
  for (int i = 0; i < CAN_DATA_BUFFER_SIZE; i++) {
    this->data[i] = data[i];
  }
}

unsigned char *CANManager::getCANData() {
    return data;
}

