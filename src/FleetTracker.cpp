// CWD-- Fleet Tracker
#include "GPSManager.h"
#include "DisplayManager.h"
#include "CANManager.h"

#define FULL_DISPLAY_TEST_ON false
#define SCREEN_REFRESH_RATE 1000000
#define CELL_GPS_REFRESH_RATE 15000000
#define GPS_REFRESH_RATE 5000000
#define GPS_DRIFT_WINDOW 45000000

bool DEBUG_ON = true;

unsigned long lastGPSPublishTime = 0;
unsigned long lastCANPublishTime = 0;

GPSManager *gpsManager = nullptr;
DisplayManager *displayManager = nullptr;
CANManager *canManager = nullptr;

/* CAN SEND TESTING CONSTS */
const uint8_t SERVICE_CURRENT_DATA = 0x01; // also known as mode 1

// These are the CAN IDs (11-bit) for OBD-II requests to the primary ECU
// and the CAN ID for the response.
const uint32_t OBD_CAN_REQUEST_ID = 0x7DF;
const uint32_t OBD_CAN_REPLY_ID = 0x7E8;

// Note: SAE PID codes are 8 bits. Proprietary ones are 16 bits.
const uint8_t PID_ENGINE_RPM = 0x0C;
const uint8_t PID_VEHICLE_SPEED = 0x0D;

byte canSendData[8] = {0x02, SERVICE_CURRENT_DATA, PID_ENGINE_RPM, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc};
/* END CAN SEND TESTING CONSTS */

SYSTEM_THREAD(ENABLED);
SerialLogHandler logHandler(LOG_LEVEL_TRACE);

// CWD-- cellular geocoding callback
void geocodedlocationCallback(float lat, float lon, float accuracy)
{
  gpsManager->log("Cell Geocoded:" + String(lat) + "," + String(lon));

  if (((gpsManager->getLongitude() == 0) && (gpsManager->getLatitude() == 0)) || (micros() - gpsManager->getLastGPSUpdate()) > gpsManager->getGPSDriftWindow())
  { // CWD-- been too long since we updated off GPS. Update off the cellular position instead
    int s = (micros() - gpsManager->getLastGPSUpdate()) / 1000000;
    Log.trace("GPS hasn't been updated in " + String(s) + " seconds. Updating from cellular positioning");
    gpsManager->setAreCoordsFromGPS(false);
    gpsManager->setLongitude(lon);
    gpsManager->setLatitude(lat);
    gpsManager->setIsGPSDataReady(true);
    gpsManager->setLastGPSUpdate(micros());
  }
}

// CWD-- particle accessors
double getLong()
{
  return gpsManager->getLongitude();
}

double getPrevLong()
{
  return gpsManager->getPrevLongitude();
}

double getLat()
{
  return gpsManager->getLatitude();
}

double getPrevLat()
{
  return gpsManager->getPrevLatitude();
}

double getAlt()
{
  return gpsManager->getAltitude();
}

double getSpeed()
{
  return gpsManager->getSpeed();
}

String getDate()
{
  return gpsManager->getDate();
}

String getTime()
{
  return gpsManager->getTime();
}

int getSatellitesCount()
{
  return gpsManager->getSatellitesCount();
}

unsigned long lastGPSUpdate()
{
  return gpsManager->getLastGPSUpdate();
}

bool areCoordsFromGPS()
{
  return gpsManager->areCoordsFromGPS();
}


// CWD-- processing
String formatDecimal(double f) {
  return String(f, 3);
}

void updateDisplay() {
  time_t time = Time.now();
  String str = Time.format(time, TIME_FORMAT_ISO8601_FULL) + "\n";
  str += formatDecimal(gpsManager->getLatitude()) + "," + formatDecimal(gpsManager->getLongitude()) + (gpsManager->areCoordsFromGPS() ? "(g)" : "(c)") + "\n";
  str+="Alt:" + formatDecimal(gpsManager->getAltitude())+"\n";
  str+="Sat Count:" + String(gpsManager->getSatellitesCount())+"\n";
  displayManager->update(str);
  //   delay(2000);
}

// CWD-- Setup
void setup()
{
  Serial.begin(9600);
  Log.info("Firing up!");

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

  Log.info("Display setup...");
  displayManager = new DisplayManager(SCREEN_REFRESH_RATE, FULL_DISPLAY_TEST_ON);
  Log.info("done.\nGPS setup...");
  gpsManager = new GPSManager(geocodedlocationCallback, GPS_REFRESH_RATE, CELL_GPS_REFRESH_RATE, GPS_DRIFT_WINDOW, false);
  Log.info("done.\nCAN setup...");
  canManager = new CANManager(CAN0_DEFAULT_INT, CAN0_DEFAULT_CS, DEBUG_ON);
  Log.info("done.");
  Log.info("System ready!");
  lastGPSPublishTime = millis();
  lastCANPublishTime = millis();
}

// CWD-- main loop
void loop()
{
  gpsManager->update();
  canManager->update();

  if ((millis() - lastCANPublishTime) > 5000) {
    byte sndStat = canManager->sendData(OBD_CAN_REQUEST_ID, 0, 8, canSendData);
    Log.trace("CAN send status: %d", sndStat);
    lastCANPublishTime = millis();
  }

  // if (Particle.connected() && canManager->isCANDataReady()) {
  //   String str = "{\"data\": [";
  //   char strTemp[64];
  //   unsigned char *canData = canManager->getCANData();
  //   int len = CAN_DATA_BUFFER_SIZE;

  //   for (byte i = 0; i < len; i++)
  //   {
  //     sprintf(strTemp, "\"0x%.2X\",", canData[i]);
  //     str += strTemp;
  //   }

  //   str.remove(str.length() - 1);

  //   str += "]}";
  
  //   if((millis() - lastCANPublishTime) > 5000) {
  //     Log.trace("Publishing CAN data");
  //     Log.trace(str);
  //     Particle.publish("CAN_data", str);
  //     Log.trace("Published CAN data");
  //     canManager->setCANDataReady(false);
  //     lastCANPublishTime = millis();
  //   } else {
  //     Log.trace("Not publishing CAN data yet. Waiting...");
  //   }
  // }

  if (Particle.connected() && ((millis() - lastGPSPublishTime) > 5000))
  {
    String strData = String::format("{ \"longitude\": %f, \"latitude\": %f, \"altitude\": %f, \"speed\": %f, \"satellites\": %d, \"date\": \"%s\", \"time\": \"%s\" }", gpsManager->getLongitude(), gpsManager->getLatitude(), gpsManager->getAltitude(), gpsManager->getSpeed(), gpsManager->getSatellitesCount(), gpsManager->getDate().c_str(), gpsManager->getTime().c_str());
    Log.trace(strData);
    Log.trace("Publishing GPS data: %s", strData.c_str());
    bool success;
    success = Particle.publish("gps_data", strData.c_str());

    if(success)
      Log.trace("Published GPS data");
    else
      Log.error("Failed to publish GPS data");

    lastGPSPublishTime = millis();
  }

  updateDisplay();
  delay(100);
}