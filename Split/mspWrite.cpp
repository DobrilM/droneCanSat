#include "config.h"
#include "mspBase.h"
#include "mspWrite.h"
#include "global.h"

void rcSetup() {	
  for (int i = 0; i < 16; i++) {
    rcValues[i] = 1500;
  }
  rcValues[2] = 1000; //throttle needs to be low for arming
}


void rcWrite() {
    mspCmd(RC_CMD, (uint8_t*)rcValues, 32);
}




