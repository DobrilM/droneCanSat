#include "config.h"
#include "bmp.h"
#include <Adafruit_BMP3XX.h>
#include <SPI.h>

Adafruit_BMP3XX bmp;
void bmpSetup() {
  if (!bmp.begin_SPI(BMP_CS)) {  
  Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
}

bmpData bmpRead() {
	static bmpData read;
	if (bmp.performReading()) {
		read.temperature = bmp.readTemperature();
		read.pressure = bmp.readPressure()/100.0;
		read.altitude = 44330.0 * (1.0 - pow(read.pressure / PRESSURE_SEA, 0.1903));

	}
	return read;
}
