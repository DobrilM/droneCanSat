#pragma once
#include <cstdint>

extern uint16_t rcValues[16];

//gps reading
extern uint8_t fix, numSat;
extern uint32_t latitude, longitude;
extern int16_t altGPS;

//battery reading
extern uint16_t battVolt;

//reading mission status
extern uint8_t navStat;

extern float accY;


