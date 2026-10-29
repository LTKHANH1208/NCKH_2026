#ifndef GPS_H
#define GPS_H

#include<Arduino.h>
#include<TinyGPS++.h>

class GPS{

private:
    HardwareSerial &gpsSerial;
    TinyGPSPlus gps;
    bool originSet;
    double lat0;
    double lon0;
    double latitude;
    double longitude;
    double x, y;
    static constexpr double EARTH_RADIUS = 6378137.0;

public:
    GPS(HardwareSerial &serial);    
    void begin(uint32_t baud = 9600);
    bool update();
    double getLatitude() const;
    double getLongitude() const;
};

#endif