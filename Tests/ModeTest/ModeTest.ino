#include <Adafruit_BMP3XX.h>

#define BMP_CS 16
#define PRESSURE_SEA 1005.7
Adafruit_BMP3XX bmp;

uint8_t status = 0;

float temperature, height, altitude;

unsigned long lastSerial = 0;


//rc send sim
int16_t 
void setup() {
  Serial.begin(9600);

  if (!bmp.begin_SPI(BMP_CS)) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
}



void standbyMode(float h, int16_t hGPS, float a) {
  if (hGPS > 1000 && a > 1 || h > 10 && a>1) { //a is in g-force. hGPS in raw values
    status = 1;
    launched = 1;
  }
}

void ascending(float h, float a) {
  if (h - lastHeight < 0 && a < 0) {
    status =2;
  }
  lastHeight = h;
}

void descending(float h, int16_t hGPS, float a, unsigned long now) {
  if (h <900 /*|| hGPS <90000 && hGPS != 0 */) { //raw gps data, if there is no fix hgps defaults to 0
    rcValues[4] = 2000; //arm, ch5, high
    status = 3;
    beforeFix = now;
  }
}

void waitForFix(unsigned long now) {
  rcValues[5] = 2000; //altHold
  if (now - beforeFix > 10000) { //10 s
    status = 5;
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
  }
}




void loop() {
  unsigned long now = millis();
  if (now - lastMspRead >= 100) {
    mspReadGPS();
    lastMspRead = now;
  }
  if (bmp.performReading()) {
    temperature = bmp.readTemperature();
    pressure = bmp.readPressure() / 100.0;   //hPa
    altitude =  44330.0 * (1.0 - pow(pressure / PRESSURE_SEA, 0.1903));
  }
  switch (state) {
      case 0: standbyMode(altitude, altGPS, accY); break;
      case 1: ascending(altitude, accY); break;
      case 2: descending(altitude, altGPS, accY, now); break;
      case 3: waitForFix(now); break;
      case 4: rtwpMode(); break;
      case 5: land(altitude); break;
  }
  if (now - lastMSP >= 20) {
    mspCmd(RC_CMD, (uint8_t*)rcValues, 32);
    lastMSP = now;
  }

  if (now - lastSerial) {
    char message[64];
    sprintf(message, "%.2f;%.2f;%.2f", temperature, pressure, altitude);
    Serial.print(message);
  }
}