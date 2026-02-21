#include "mspBase.h"
#include "mspWrite.h"
#include "config.h"


//reading gps
void mspReadGPS() {
  mspCmd(GPS_GET, nullptr, 0);
  while (Serial1.available()) {
    parseMSP(Serial1.read());
  }
}

//reading battery voltage
void mspReadVoltage() {
  mspCmd(BATT_GET, nullptr, 0);
  while (Serial1.available()) {
    parseMSP(Serial1.read());
  }
}

//reading mission status
void mspReadMissionStatus() {
  mspCmd(NAV_STAT, nullptr, 0);
  while (Serial1.available()) {
    parseMSP(Serial1.read());
  }
}

void mspRead() {
	mspReadGPS();
	mspReadVoltage();
	mspReadMissionStatus();
}
