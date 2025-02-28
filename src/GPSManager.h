
#pragma once
#ifndef __GPSManager_h
#define __GPSManager_h

#include <TinyGPS++.h>
#include <locator.h>

#define ss Serial1
#define CELL_GPS_PERIODIC_PUBLISH_INTERVAL 120
const String PUB_PREFIX = "deviceLocation_";

class GPSManager {
  public:
    explicit GPSManager(LocatorSubscriptionCallback googleCallback, unsigned long gpsRefreshInterveral, unsigned long cellRefreshInterveral,
                        unsigned long gpsDriftWindow, bool debugOn = false);
    ~GPSManager();

    void update();
    void processData();
    void checkGPS();

    // CWD-- getters
    bool areCoordsFromGPS();
    bool setAreCoordsFromGPS(bool areCoordsFromGPS);
    bool isGPSDataReady();
    bool setIsGPSDataReady(bool isReady);
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

    static double haversine(double lat1, double lon1, double lat2, double lon2);

  private: // Private members
    bool blnDebugOn = false;
    bool blnCoordsFromGPS = false;
    bool blnGPSDataReady = false;
    double dblLongitude = 0;
    double dblLatitude = 0;
    double dblPrevLongitude = 0;
    double dblPrevLatitude = 0;
    double dblDistanceMoved = 0;

    double dblAltitude = 0;
    double dblSpeed = 0;

    int iSatellitesCount = 0;
    unsigned long ulLastScreenUpdate;
    unsigned long ulLastCellGPSUpdate = 0;
    unsigned long ulLastGPSUpdate = 0;

    unsigned long ulGPSRefreshInterveral = 0;
    unsigned long ulCellRefreshInterveral = 0;
    unsigned long ulGPSDriftWindow = 0;

    String strDate;
    String strTime;

    // LocatorSubscriptionCallback googleCallback;

    // GPS objects
    TinyGPSPlus gps;
    Locator locator;
    // void GPSManager::geocodedlocationCallback(float lat, float lon, float accuracy);
};

#endif // def(__GPSManager_h)