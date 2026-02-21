#include "blink.h"
#include "timing.h"
#include "config.h"
#include "bmp.h"
#include "imu.h"
#include "mspRead.h"
#include "mspWrite.h"
#include "global.h"
#include "geozone.h"

void setup() {
  blinkSetup();
  bmpSetup();
  imuSetup();
  Serial1.begin(9600);
  rcSetup();
}

void loop() {
  unsigned long now = millis();
  bmpData bmpData = bmpRead();
  if (timingSens(now)) {
    geozoneCheck();
    accY = imuRead();
    mspRead();
  }
  if(timingRadio(now)) {
    blink();
  }
  if(timingMSP(now)) {
    rcWrite();
  }
}
