#include "CANManager.h"
#include <SPI.h>
#include <Wire.h>
#include <mcp_can.h>

CANManager::CANManager(int IntPin, int CSPin, bool debugOn) : iIntPin(IntPin), iCSPin(CSPin), blnDebugOn(debugOn) {
    pinMode(iIntPin, INPUT);    // INPUT_PULLUP  // Configuring pin for /INT input
    CAN0 = new MCP_CAN(iCSPin); // Set CS for CAN0
    byte canStatus = CAN0->begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);

    if (canStatus == CAN_OK) {
        CAN0->setMode(MCP_NORMAL); // Set operation mode to normal so the MCP2515 sends acks to received data.
        blnCANInitialized = true;

        if (blnDebugOn) {
            Log.trace("MCP2515 Initialized Successfully!");
        }
    } else {
        blnCANInitialized = false;

        if (blnDebugOn) {
            Log.error("Error Initializing MCP2515: %d", canStatus);
        }
    }
}

CANManager::~CANManager() {
    // Nothing to do here
}

void CANManager::update() {
    if (blnCANInitialized) {
        Log.trace("CAN was initialized correctly. Checking for data...");

        if (!digitalRead(iIntPin)) { // If iIntPin pin is low, read receive buffer
            Log.trace("Data available. Reading data...");
            CAN0->readMsgBuf(&rxId, &len, data); // Read data: len = data length, buf = data byte(s)
            Log.trace("Raw ID:" + String(rxId));

            if (blnDebugOn) {
                char msgString[128];

                if ((rxId & 0x80000000) == 0x80000000) { // Determine if ID is standard (11 bits) or extended (29 bits)
                    sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
                } else {
                    sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
                }

                log(msgString);

                if ((rxId & 0x40000000) == 0x40000000) { // Determine if message is a remote request frame.
                    sprintf(msgString, " REMOTE REQUEST FRAME");
                    // log(msgString);
                } else {
                    log("[");

                    for (byte i = 0; i < len; i++) {
                        sprintf(msgString, " 0x%.2X", data[i]);
                        log(msgString, false);
                    }

                    log("]", true);
                }
            }

            if (len > 0)
                blnCANDataReady = true;
            else
                blnCANDataReady = false;
        } else {
            blnCANDataReady = false;
            Log.trace("No data available. Exiting update...");
        }
    } else {
        Log.trace("CAN not initialized. Exiting update...");
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

void CANManager::setDebug(bool blnDebug) { blnDebugOn = blnDebug; }

bool CANManager::getDebug() { return blnDebugOn; }

bool CANManager::isCANDataReady() { return blnCANInitialized && blnCANDataReady; }

void CANManager::setCANDataReady(bool blnCANDataReady) { this->blnCANDataReady = blnCANDataReady; }

void CANManager::setCANData(unsigned char *data) {
    for (int i = 0; i < CAN_DATA_BUFFER_SIZE; i++) {
        this->data[i] = data[i];
    }
}

unsigned char *CANManager::getCANData() { return data; }
long unsigned int CANManager::getCANRxId() { return rxId; }

byte CANManager::sendData(unsigned long id, byte ext, byte len, byte *buf) {
    byte ret = CAN_FAIL;

    if (blnCANInitialized) {
        Log.trace("Sending data: %d bytes to %d", len, id);
        ret = CAN0->sendMsgBuf(id, 0, len, buf);
        Log.trace("Data sent. Status: %d", ret);
    }

    return ret;
}
