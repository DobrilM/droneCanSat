#include "MSP.h"

MSP msp;
msp_set_raw_rc_t rc;
msp_raw_gps_t gps;
msp_analog_t batt;
msp_nav_status_t navStat;
uint8_t fix;
uint8_t numSat;
uint32_t rawLat;
uint32_t rawLong;
int16_t gpsAlt;

uint16_t battVolt;

uint8_t navMode;
uint8_t navState;
uint8_t navError;

float latitude;
float longitude;

uint8_t MSPReqTurn = 0;

void setup() {
  Serial1.begin(115200);
  msp.begin(Serial1);
  for (int i=0; i < MSP_MAX_SUPPORTED_CHANNELS; i++) {
    rc.channel[i] = 1500;
  }
  rc.channel[2] = 1000;
}

void loop() {
  unsigned long now = millis();

  static unsigned long start = now;
  static unsigned long lastMSP = 0;
  static unsigned long lastMSPReq = 0;
  static unsigned long lastSerial = 0;

  if (now - start > 3000) {
    rc.channel[5] = 2000;
  }
  
  if (now - lastMSPReq > 25) {
    switch (MSPReqTurn) {
      case 0: 
        if (msp.request(MSP_RAW_GPS, &gps, sizeof(gps))) {
          fix = gps.fixType;
          numSat = gps.numSat;
          rawLat = gps.lat;
          rawLong = gps.lon;
          latitude = rawLat / 1e7f;
          longitude = rawLong / 1e7f; 
          gpsAlt = gps.alt;
        }
        MSPReqTurn = 1;
        break;
      case 1:
        if(msp.request(MSP_ANALOG, &batt, sizeof(batt))) {
          battVolt = batt.vbat;
        }
        MSPReqTurn = 2;
        break;
      case 2:
        if(msp.request(MSP_NAV_STATUS, &navStat, sizeof(navStat))) {
          navMode = navStat.mode;
          navState = navStat.state;
          navError = navStat.error;
        }
        MSPReqTurn = 0;
        break;
    }
    lastMSPReq = now;
  }

  if (now - lastSerial > 1000) {
    char message[128];
    sprintf(message, "%i;%i;%i;%i;%.7f;%.7f;%i;%i;%i;%i;%i", fix, numSat, rawLat, rawLong, latitude, longitude, gpsAlt, battVolt, navMode, navState, navError);
    Serial.println(message);
    lastSerial = now;
  }

  if (now - lastMSP > 50) {
    msp.command(MSP_SET_RAW_RC, &rc, sizeof(rc));
    lastMSP = now;
  }
}