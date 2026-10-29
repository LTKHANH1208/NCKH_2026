#include"gps.h"
#include<math.h>

GPS::GPS(HardwareSerial &serial) : gpsSerial(serial)
{
    originSet = false;
    lat0 = 0;
    lon0 = 0;

    latitude = 0;
    longitude = 0;

    x = 0;
    y = 0;
}

void GPS::begin(uint32_t baud){
    gpsSerial.begin(baud);
}

bool GPS::update(){
    while (gpsSerial.available())
    {
        gps.encode(gpsSerial.read());
    }
    
    if(!gps.location.isUpdated()) return false;

    latitude = gps.location.lat();
    longitude = gps.location.lng();
    if(!originSet){
        lat0 = latitude;
        lon0 = longitude;
        originSet = true;
    }
    double dLat = (latitude - lat0) * M_PI / 180.0;
    double dLon = (longitude - lon0) * M_PI / 180.0;

    double lat0Rad = lat0 * M_PI / 180;

    x = dLon * EARTH_RADIUS * cos(lat0Rad);
    y = dLat * EARTH_RADIUS;

    return true;
}
