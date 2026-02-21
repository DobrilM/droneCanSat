#include "config.h"
#include "global.h"
#include "mode.h"

void standbyMode(float h) {
	if (altGPS > 1000 && accY > 1 || h > 10 && accY>1) { //a is in g-force. hGPS in raw values
		status = 1;
		launched = 1;
	}
}

void ascending(float h) {
	static float lastHeight = 0.0;
	if (h - lastHeight < 0 && accY < 0) {
		status =2;
	}
	lastHeight = h;
}

void descending(float h, unsigned long now) {
	if (h <900 || (altGPS <90000 && altGPS != 0)) { //raw gps data, if there is no fix hgps defaults to 0
		rcValues[4] = 2000; //arm, ch5, high
		status = 3;
		beforeFix = now;
	}
}

void waitForFix(unsigned long now) {
	rcValues[5] = 2000; //altHold
	if (now - beforeFix > 10000) { //10 s
		status =5;
	}
	if (fix > 0) {
		status = 4;
	}
}

void rtwpMode() {
	rcValues[5] = 1000; //althold off
	rcValues[6] = 2000; //ch 7, set to navigate mission
	if (navStat == 0) {
		status = 5;
	}
}

void land(float h) {
	rcValues[5] = 1000; //turn off althold
	rcValues[6] = 1000; //turn off rtwp
	if (h>10) {
		rcValues[2] = 1300; //slowed descent
	} else {
		rcValues[2] = 1000;//throttle low and disarm
		rcValues[4] = 1000;
		rcValues[7] = 2000; //beeper
	}
}

void modeRun(float h, unsigned long now) {
	switch (status) {
		case 0: standbyMode(h); break;
		case 1: ascending(h); break;
		case 2: descending(h, now); break;
		case 3: waitForFix(now); break;
		case 4: rtwpMode(); break;
		case 5: land(h); break;
		default: 
			while(1) {
				delay(100);
			}
			break;
	}
}
