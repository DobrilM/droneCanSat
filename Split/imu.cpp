#include "config.h"
#include <Adafruit_BNO08x.h>

Adafruit_BNO08x imu(BNO08X_RST);
sh2_SensorValue_t sensorValue;

void imuSetup() {
	if(!imu.begin_SPI(BNO08X_CS, BNO08X_INT)) {
		while(1);
	}
}

float imuRead() {
	imu.enableReport(SH2_ACCELEROMETER);
	return sensorValue.un.accelerometer.y;
}
