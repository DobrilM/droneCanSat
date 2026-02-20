#include "blink.h"
#include "config.h"


void blinkSetup() {
	pinMode(LED_BUILTIN, OUTPUT);
	blink();
}

void blink() {
	digitalWrite(LED_BUILTIN, HIGH);
	delay(10);
	digitalWrite(LED_BUILTIN, HIGH);
	delay(10);
}
