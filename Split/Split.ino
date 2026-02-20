#include "blink.h"
#include "timing.h"
#include "config.h"
#include "bmp.h"
#include "imu.h"


void setup() {
  blinkSetup();
  bmpSetup();
  imuSetup();
}

void loop() {
  unsigned long now = millis();
  bmpRead();
  if (timingSens(now)) {
    imuRead();
  }
  if(timingRadio(now)) {
    blink();
  }
}
