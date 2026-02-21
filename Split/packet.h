#pragma once

#include <cstdint>
#include "bmp.h"
struct message {
  int16_t temp;
  int16_t alt; 
  int32_t pressure;
  uint8_t fix;
  uint8_t numSat;
  uint32_t latitude;
  uint32_t longitude;
  int16_t altGPS;
  uint16_t battVolt;
  uint8_t navStat;
  uint8_t geozoneStat;
  uint8_t status;
};

message makeMessage(bmpData bmpData);
