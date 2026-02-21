#pragma once
#include <cstdint>


constexpr uint8_t GPS_GET = 106;
constexpr uint8_t NAV_STAT = 121;
constexpr uint8_t BATT_GET = 130;
constexpr uint8_t RC_CMD = 200;


void mspSetup();

void mspCmd(uint8_t cmd, uint8_t* payload, uint8_t size);

void parsePacket(uint8_t cmd);

void parseMSP(uint8_t readChar);
