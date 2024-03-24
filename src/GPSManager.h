
#pragma once
#ifndef __GPSManager_h
#define __GPSManager_h

#include <TinyGPS++.h>
#include <google-maps-device-locator.h>

#define ss Serial1
#define CELL_GPS_PERIODIC_PUBLISH_INTERVAL 120
const String PUB_PREFIX = "deviceLocation_";

class GPSManager {
  public:
    explicit GPSManager(GoogleMapsDeviceLocatorSubscriptionCallback googleCallback, unsigned long gpsRefreshInterveral, unsigned long cellRefreshInterveral, unsigned long gpsDriftWindow, bool debugOn = false);
    ~GPSManager();

    void update();
    void processData();
    void checkGPS();

    //CWD-- getters
    bool areCoordsFromGPS();
    bool setAreCoordsFromGPS(bool areCoordsFromGPS);
    double getLongitude();
    double setLongitude(double longitude);
    double getPrevLongitude();
    double getLatitude();
    double setLatitude(double latitude);
    double getPrevLatitude();
    double getAltitude();
    double getSpeed();
    int getSatellitesCount();
    unsigned long getLastGPSUpdate();
    unsigned long setLastGPSUpdate(unsigned long lastGPSUpdate);
    unsigned long getGPSRefreshInterveral();
    unsigned long getCellRefreshInterveral();
    unsigned long getGPSDriftWindow();
    double getDistanceMoved();
    String getDate();
    String getTime();
    void setDebug(bool blnDebug);
    void log(String str, bool blnWithNewLine = false);

    static double GPSManager::haversine(double lat1, double lon1, double lat2, double lon2);

  private : // Private members
    bool blnDebugOn = false;
    bool blnCoordsFromGPS = false;
    double dblLongitude = 0;
    double dblLatitude = 0;
    double dbPrevLongitude = 0;
    double dbPrevLatitude = 0;
    double dblDistanceMoved = 0;

    double dblAltitude = 0;
    double dblSpeed = 0;

    int iSatellitesCount = 0;
    unsigned long lastScreenUpdate;
    unsigned long lastCellGPSUpdate = 0;
    unsigned long lastGPSUpdate = 0;

    unsigned long ulGPSRefreshInterveral = 0;
    unsigned long ulCellRefreshInterveral = 0;
    unsigned long ulGPSDriftWindow = 0;

    String strDate;
    String strTime;

    // GoogleMapsDeviceLocatorSubscriptionCallback googleCallback;

    // GPS objects
    TinyGPSPlus gps;
    GoogleMapsDeviceLocator locator;
    // void GPSManager::geocodedlocationCallback(float lat, float lon, float accuracy);
};

#endif // def(__GPSManager_h)