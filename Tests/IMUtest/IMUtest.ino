#include <Adafruit_BNO08x.h>

#define BNO08X_INT 17
#define BNO08X_RST 18
#define BNO08X_CS 19

Adafruit_BNO08x imu(BNO08X_RST);
sh2_SensorValue_t sensorValue;

unsigned long lastIMU = 0;

unsigned long lastSerial = 0;
void setup() {
  Serial.begin(9600);
  while (!Serial);

  if(imu.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    if (!imu.enableReport(SH2_ACCELEROMETER)) {
      Serial.println("Could not enable accelerometer");
    }
  } else {
    Serial.print("imu not readable");
  }

}

void loop() {
  unsigned long now = millis();
  if (now - lastIMU >= 100 && imu.wasReset()) {
    Serial.print("sensor was reset ");
      if (!imu.enableReport(SH2_ACCELEROMETER)) {
      Serial.println("Could not enable accelerometer");
    }
    lastIMU = now;
  }
  if (now - lastSerial >=1000) {
  Serial.print("Accelerometer - x: ");
  Serial.print(sensorValue.un.accelerometer.x);
  Serial.print(" y: ");
  Serial.print(sensorValue.un.accelerometer.y);
  Serial.print(" z: ");
  Serial.println(sensorValue.un.accelerometer.z);
  lastSerial = now;
  }
}