#include "timing.h"
#include "config.h"
bool timingRadio(unsigned long now) {
	static unsigned long lastRadio = 0;
	if (now - lastRadio >= 1000 ) {
		lastRadio = now;
		return true;
	} else {
		return false;
	}
}

bool timingSensor(unsigned long now) {
	static unsigned long lastSensor = 0;
	if (now - lastSensor >= 100 ) {
		lastSensor = now;
		return true;
	} else {
		return false;
	}
}

bool timingMSP(unsigned long now) {
	static unsigned long lastMSP = 0;
	if (now - lastMSP >= 50 ) {
		lastMSP = now;
		return true;
	} else {
		return false;
	}
}
