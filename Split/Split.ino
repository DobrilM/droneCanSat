#include "blink.h"
#include "timing.h"




void setup() {
  blinkSetup();
}

void loop() {
  unsigned long now = millis();
  if(timingRadio(now)) {
    blink();
  }
}
