#include <SPI.h>
#include <Wire.h>

#include <mcp_can.h>
#include <Adafruit_GFX_RK.h>
#include <Adafruit_SSD1306_RK.h>

#include "GPSManager.h"
#include "DisplayManager.h"

#define FULL_DISPLAY_TEST_ON false
#define SCREEN_REFRESH_RATE 1000000
#define CELL_GPS_REFRESH_RATE 15000000
#define GPS_REFRESH_RATE 5000000
#define GPS_DRIFT_WINDOW 45000000

bool DEBUG_ON = true;

GPSManager gpsManager;
DisplayManager displayManager;

// CWD-- CAN Bus
SYSTEM_THREAD(ENABLED);
SerialLogHandler logHandler;

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128]; // Array to store serial string

#define CAN0_INT A1 // Set INT to pin A1
MCP_CAN CAN0(A2);   // Set CS to pin A2

// CWD-- globals to track state
bool blnCANDataReady = false;

void geocodedlocationCallback(float lat, float lon, float accuracy)
{
  gpsManager.log("Cell Geocoded:" + String(lat) + "," + String(lon));

  if (((gpsManager.getLongitude() == 0) && (gpsManager.getLatitude() == 0)) || (micros() - gpsManager.getLastGPSUpdate()) > gpsManager.getGPSDriftWindow())
  { // CWD-- been too long since we updated off GPS. Update off the cellular position instead
    int s = (micros() - gpsManager.getLastGPSUpdate()) / 1000000;
    gpsManager.log("GPS hasn't been updated in " + String(s) + " seconds. Updating from cellular positioning");
    gpsManager.setAreCoordsFromGPS(false);
    gpsManager.setLongitude(lon);
    gpsManager.setLatitude(lat);
  }
}

// CWD-- particle accessors
double getLong()
{
  return gpsManager.getLongitude();
}

double getPrevLong()
{
  return gpsManager.getPrevLongitude();
}

double getLat()
{
  return gpsManager.getLatitude();
}

double getPrevLat()
{
  return gpsManager.getPrevLatitude();
}

double getAlt()
{
  return gpsManager.getAltitude();
}

double getSpeed()
{
  return gpsManager.getSpeed();
}

String getDate()
{
  return gpsManager.getDate();
}

String getTime()
{
  return gpsManager.getTime();
}

int getSatellitesCount()
{
  return gpsManager.getSatellitesCount();
}

unsigned long lastGPSUpdate()
{
  return gpsManager.getLastGPSUpdate();
}

bool areCoordsFromGPS()
{
  return gpsManager.areCoordsFromGPS();
}

void updateCAN()
{
  if (!digitalRead(CAN0_INT)) // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)

    if ((rxId & 0x80000000) == 0x80000000) // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    else
      sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);

    Serial.print(msgString);

    if ((rxId & 0x40000000) == 0x40000000)
    { // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    }
    else
    {
      for (byte i = 0; i < len; i++)
      {
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.print(msgString);
      }
    }

    Serial.println();
    blnCANDataReady = true;
  }
}

// CWD-- processing
String formatDecimal(double f) {
  return String(f, 3);
}

void updateDisplay()
{
  String str=gpsManager.getDate() + " " + gpsManager.getTime() + "\n";
  str+=formatDecimal(gpsManager.getLongitude()) + "," + formatDecimal(gpsManager.getLatitude()) + (gpsManager.areCoordsFromGPS() ? "(g)" : "(c)")+"\n";
  str+="Alt:" + formatDecimal(gpsManager.getAltitude())+"\n";
  str+="Sat Count:" + String(gpsManager.getSatellitesCount())+"\n";
  displayManager.update(str);
  //   delay(2000);
}

// CWD-- Setup
void setup()
{
  Serial.begin(9600);
  Serial.println("Firing up!");

  Particle.variable("longitude", getLong);
  Particle.variable("latitude", getLat);
  Particle.variable("prevLongitude", getPrevLong);
  Particle.variable("prevLatitude", getPrevLat);
  Particle.variable("altitude", getAlt);
  Particle.variable("speed", getSpeed);
  Particle.variable("date", getDate);
  Particle.variable("time", getTime);
  Particle.variable("satellitesCount", getSatellitesCount);
  Particle.variable("lastGPSUpdate", lastGPSUpdate);
  Particle.variable("coordsFromGPS", areCoordsFromGPS);

  displayManager = DisplayManager(SCREEN_REFRESH_RATE, FULL_DISPLAY_TEST_ON);
  gpsManager = GPSManager(geocodedlocationCallback, GPS_REFRESH_RATE, CELL_GPS_REFRESH_RATE, GPS_DRIFT_WINDOW, DEBUG_ON);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL); // Set operation mode to normal so the MCP2515 sends acks to received data.
  pinMode(CAN0_INT, INPUT); // Configuring pin for /INT input
  Serial.println("MCP2515 CAN setup...");
  Serial.println("System ready!");
}

// CWD-- main loop
void loop()
{

  gpsManager.update();
  updateCAN();

  if (blnCANDataReady && Particle.connected())
  {
    String str = "{\"data\": [";

    for (byte i = 0; i < len; i++)
    {
      sprintf(msgString, "\"0x%.2X\",", rxBuf[i]);
      str += msgString;
    }

    str.remove(str.length() - 1);

    str += "]}";
    // log("Publishing CAN data");
    // log(str);
    Particle.publish("CAN_data", str);
    // log("Published CAN data");
    blnCANDataReady = false;
  }
}