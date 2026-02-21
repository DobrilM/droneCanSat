#include "config.h"
#include "packet.h"
#include "global.h"

message makeMessage(bmpData bmpData) {
  message p{};
  p.temp = bmpData.temperature * 100; //conv to int
  p.alt = bmpData.altitude * 100;
  p.pressure = bmpData.pressure * 100;
  p.fix = fix;
  p.numSat = numSat;
  p.latitude = latitude;
  p.longitude = longitude;
  p.altGPS = altGPS;
  p.battVolt = battVolt;
  p.navStat = navStat;
  p.geozoneStat = geozoneCheck;
  p.status = status;
  return p;
};
