#include <math.h>
#include "GPSManager.h"
#include <TinyGPS++.h>
#include <google-maps-device-locator.h>

GPSManager::GPSManager(GoogleMapsDeviceLocatorSubscriptionCallback googleCallback, unsigned long gpsRefreshInterveral, unsigned long cellRefreshInterveral, unsigned long gpsDriftWindow, bool debugOn = false) : blnDebugOn(debugOn)
{ // Constructor
  ulGPSRefreshInterveral = gpsRefreshInterveral;
  ulCellRefreshInterveral = cellRefreshInterveral;
  ulGPSDriftWindow = gpsDriftWindow;
  // CWD-- start serial for GPS
  ss.begin(9600);

  // CWD-- config Particle Google Integration to be on demand
  log("Setting cell gps event name to: " + PUB_PREFIX + "cell");
  locator.withEventName(PUB_PREFIX + "cell");
  locator.publishLocation();
  // auto googleCallback = [this](float lat, float lon, float accuracy) { this->geocodedlocationCallback(lat, lon, accuracy); };
  locator.withSubscribe(googleCallback)
      .withLocatePeriodic(CELL_GPS_PERIODIC_PUBLISH_INTERVAL);
}

GPSManager::~GPSManager()
{ // Destructor
}

void GPSManager::update()
{
  // CWD-- Particle Google Maps Integration call
  locator.loop();

  checkGPS(); // CWD-- run through the serial buffer and ingest the data

  if ((micros() - lastCellGPSUpdate) > ulCellRefreshInterveral) { // CWD-- update the location via cell geocoding
    if (Particle.connected()) {
      log("Connected to particle...");
      log("Publishing Celluar GPS Locator...");
      locator.publishLocation();
      lastCellGPSUpdate = micros();
      log("Published Celluar GPS Locator event");
    } else {
      log("Can't connect to particle"); // CWD-- need to buffer data
    }
  }

  if ((micros() - lastGPSUpdate) > ulGPSRefreshInterveral) { // CWD-- update the location via GPS
    if (Particle.connected()) {
      if ((dblLongitude != dbPrevLongitude) && (dblLatitude != dbPrevLatitude)) { //(dblLongitude!=0)&&(dblLatitude!=0)&&
        log("Publishing real GPS coords...");
        char locationBeacon[256];
        snprintf(locationBeacon, sizeof(locationBeacon), "{\n\t\"longitude: %f\",\n\t\"latitude\": %f,\n\t\"altitude\": %f\n }", dblLongitude, dblLatitude, dblAltitude);
        log(locationBeacon);
        Particle.publish(PUB_PREFIX + "gps", locationBeacon);
        log("Published real GPS coords.");
        lastGPSUpdate = micros();
      } else {
        // writeDebug("Real GPS location has not changed. Not publishing");
      }
    } else {
      log("Can't connect to particle"); // CWD-- need to buffer data
    }
  }
}

void GPSManager::log(String str, bool blnWithNewLine = false)
{
  if (blnDebugOn)
  {
    Serial.print(str);

    if (blnWithNewLine)
    {
      Serial.print('\n');
    }
  }
}

void GPSManager::processData()
{ // Update GPS data
  if (gps.location.isValid() && gps.location.isUpdated())
  {
    if (blnDebugOn)
    {
      log("Previous Latitude= ");
      log(String(dblLatitude, 6));
      log("Previous  Longitude= ");
      log(String(dbPrevLongitude, 6));
    }
    // Latitude in degrees (double)
    blnCoordsFromGPS = true;
    dbPrevLongitude = dblLongitude;
    dbPrevLatitude = dblLatitude;
    dblLongitude = gps.location.lng();
    dblLatitude = gps.location.lat();
    lastGPSUpdate = micros();

    if (blnDebugOn)
    {
      log("Latitude= ", false);
      log(String(dblLatitude, 6), false);
      // Longitude in degrees (double)
      log(" Longitude= ", false);
      log(String(dblLongitude, 6));

      // Raw latitude in whole degrees
      log("Raw latitude = ", false);
      log(gps.location.rawLat().negative ? "-" : "+", false);
      log(String(gps.location.rawLat().deg));
      // ... and billionths (u16/u32)
      log(String(gps.location.rawLat().billionths));

      // Raw longitude in whole degrees
      log("Raw longitude = ", false);
      log(gps.location.rawLng().negative ? "-" : "+", false);
      log(String(gps.location.rawLng().deg));
      // ... and billionths (u16/u32)
      log(String(gps.location.rawLng().billionths));
    }
  }

  if (gps.date.isValid() && gps.date.isUpdated())
  {
    // Raw date in DDMMYY format (u32)
    strDate = gps.date.year();
    strDate += "/";
    strDate += gps.date.month();
    strDate += "/";
    strDate += gps.date.day();

    if (blnDebugOn)
    {
      log("Raw date DDMMYY = ", false);
      log(String(gps.date.value()), false);
      // Year (2000+) (u16)
      log("\tYear = ", false);
      log(String(gps.date.year()), false);
      // Month (1-12) (u8)
      log("\tMonth = ", false);
      log(String(gps.date.month()), false);
      // Day (1-31) (u8)
      log("\tDay = ", false);
      log(String(gps.date.day()), false);
    }
  }

  if (gps.time.isValid() && gps.time.isUpdated())
  {
    // Raw time in HHMMSSCC format (u32)
    strTime = gps.time.hour();
    strTime += ":";
    strTime += gps.time.minute();
    strTime += ":";
    strTime += gps.time.second();

    if (blnDebugOn)
    {
      log("Raw time in HHMMSSCC = ", false);
      log(String(gps.time.value()), false);
      // Hour (0-23) (u8)
      log("\tHour = ", false);
      log(String(gps.time.hour()), false);
      // Minute (0-59) (u8)
      log("\tMinute = ", false);
      log(String(gps.time.minute()), false);
      // Second (0-59) (u8)
      log("\tSecond = ", false);
      log(String(gps.time.second()), false);
      // 100ths of a second (0-99) (u8)
      log("\tCentisecond = ", false);
      log(String(gps.time.centisecond()));
    }
  }

  if (gps.speed.isValid() && gps.speed.isUpdated())
  {
    dblSpeed = gps.speed.mph();

    if (blnDebugOn)
    {
      // Raw speed in 100ths of a knot (i32)
      log("Raw speed in 100ths/knot = ", false);
      log(String(gps.speed.value()), false);
      // Speed in knots (double)
      log("\tSpeed in knots/h = ", false);
      log(String(gps.speed.knots()), false);
      // Speed in miles per hour (double)
      log("\tSpeed in miles/h = ", false);
      log(String(dblSpeed), false);
      // Speed in meters per second (double)
      log("\tSpeed in m/s = ", false);
      log(String(gps.speed.mps()), false);
      // Speed in kilometers per hour (double)
      log("\tSpeed in km/h = ", false);
      log(String(gps.speed.kmph()));
    }
  }

  if (gps.course.isValid() && gps.course.isUpdated())
  {
    if (blnDebugOn)
    {
      // Raw course in 100ths of a degree (i32)
      log("Raw course in degrees = ", false);
      log(String(gps.course.value()), false);
      // Course in degrees (double)
      log("\tCourse in degrees = ", false);
      log(String(gps.course.deg()));
    }
  }

  if (gps.altitude.isValid() && gps.altitude.isUpdated())
  {
    dblAltitude = gps.altitude.feet();

    if (blnDebugOn)
    {
      // Raw altitude in centimeters (i32)
      log("Raw altitude in centimeters = ", false);
      log(String(gps.altitude.value()), false);
      // Altitude in meters (double)
      log("\tAltitude in meters = ", false);
      log(String(gps.altitude.meters()));
      // Altitude in miles (double)
      log("\tAltitude in miles = ", false);
      log(String(gps.altitude.miles()), false);
      // Altitude in kilometers (double)
      log("\tAltitude in kilometers = ", false);
      log(String(gps.altitude.kilometers()), false);
      // Altitude in feet (double)
      log("\tAltitude in feet = ", false);
      log(String(dblAltitude));
    }
  }

  if (gps.satellites.isValid() && gps.satellites.isUpdated())
  {
    iSatellitesCount = gps.satellites.value();

    if (blnDebugOn)
    {
      // Number of satellites in use (u32)
      log("Number of satellites in use = ", false);
      log(String(iSatellitesCount));
      //   log("charsProcessed: ", false);
      //   log(gps.charsProcessed(), false);
      //   log("\tsentencesWithFix: ", false);
      //   log(gps.sentencesWithFix(), false);
      //   log("\tfailedChecksum: ", false);
      //   log(gps.failedChecksum(), false);
      //   log("\tpassedChecksum: ", false);
      //   log((gps.passedChecksum());
    }
  }

  if (gps.hdop.isValid() && gps.hdop.isUpdated())
  {
    if (blnDebugOn)
    {
      // Horizontal Dim. of Precision (100ths-i32)
      log("HDOP = ", false);
      log(String(gps.hdop.value()));
    }
  }
}

void GPSManager::checkGPS()
{ // Check GPS
  //   Serial.print("Waiting for GPS Data: ");
  //   Serial.println(ss.available());
  while (ss.available() > 0)
  { // CWD-- we have data on the serial
    // get the byte data from the GPS
    byte gpsData = ss.read();
    gps.encode(gpsData);
    processData();

    // if(blnDebugOn) {
    Serial.write(gpsData);
    //  }
  }
}

bool GPSManager::areCoordsFromGPS() {
  return blnCoordsFromGPS;
}

bool GPSManager::setAreCoordsFromGPS(bool areCoordsFromGPS) {
  bool t = blnCoordsFromGPS;
  blnCoordsFromGPS = areCoordsFromGPS;
  return t;
}

double GPSManager::getLongitude()
{
  return dblLongitude;
}

double GPSManager::setLongitude(double longitude)
{
  dbPrevLongitude = dblLongitude;
  dblLongitude = longitude;
  return dbPrevLongitude;
}

double GPSManager::getPrevLongitude() {
  return dbPrevLongitude;
}

double GPSManager::setLatitude(double latitude) {
  dbPrevLatitude = dblLatitude;
  dblLatitude = latitude;
  return dbPrevLatitude;
}

double GPSManager::getLatitude() {
  return dblLatitude;
}

double GPSManager::getPrevLatitude() {
  return dbPrevLatitude;
}

double GPSManager::getAltitude() {
  return dblAltitude;
}

double GPSManager::getSpeed() {
  return dblSpeed;
}

int GPSManager::getSatellitesCount() {
  return iSatellitesCount;
}

unsigned long GPSManager::getLastGPSUpdate() {
  return lastGPSUpdate;
}

unsigned long GPSManager::setLastGPSUpdate(unsigned long lastGPSUpdate)
{
  unsigned long t = lastGPSUpdate;
  lastGPSUpdate = lastGPSUpdate;
  return t;
}

unsigned long GPSManager::getGPSRefreshInterveral()
{
  return ulGPSRefreshInterveral;
}

unsigned long GPSManager::getCellRefreshInterveral()
{
  return ulCellRefreshInterveral;
}

unsigned long GPSManager::getGPSDriftWindow()
{
  return ulGPSDriftWindow;
}

double GPSManager::getDistanceMoved() {
  return dblDistanceMoved;
}

String GPSManager::getDate() {
  return strDate;
}

String GPSManager::getTime() {
  return strTime;
}

void GPSManager::setDebug(bool blnDebug) {
  blnDebugOn = blnDebug;
}

// void GPSManager::geocodedlocationCallback(float lat, float lon, float accuracy)
// {
//   log("Geocoded:" + String(lat) + "," + String(lon));

//   if (((dblLongitude == 0) && (dblLatitude == 0)) || (micros() - lastGPSUpdate) > ulGPSDriftWindow)
//   { // CWD-- been too long since we updated off GPS. Update off the cellular position instead
//     int s = (micros() - lastGPSUpdate) / 1000000;
//     log("GPS hasn't been updated in " + String(s) + " seconds. Updating from cellular positioning");
//     blnCoordsFromGPS = false;
//     dbPrevLongitude = dblLongitude;
//     dbPrevLatitude = dblLatitude;
//     dblLatitude = lat;
//     dblLongitude = lon;
//   }
// }
// CWD--
//  Returns the great-circle distance (in meters) between two points on a sphere
//  lat1, lat2, lon1, lon2 must be provided in Degrees.  (Radians = Degrees * PI / 180, Degrees = Radians / PI * 180)
double GPSManager::haversine(double lat1, double lon1, double lat2, double lon2)
{
  const double rEarth = 6371000.0; // in meters
  double x = pow(sin(((lat2 - lat1) * M_PI / 180.0) / 2.0), 2.0);
  double y = cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0);
  double z = pow(sin(((lon2 - lon1) * M_PI / 180.0) / 2.0), 2.0);
  double a = x + y * z;
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  double d = rEarth * c;
  // log(f("%12.9f, %12.9f, %12.9f, %12.9f, %12.9f, %12.9f", x, y, z, a, c, d);
  return d; // in meters
}