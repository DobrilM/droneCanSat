#include "config.h"

uint16_t rcValues[16];

//gps reading
uint8_t fix, numSat;
uint32_t latitude, longitude;
int16_t altGPS;

//battery reading
uint16_t battVolt;

//reading mission status
uint8_t navStat;

float accY;

