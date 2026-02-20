// Basic demo for readings from Adafruit BNO08x
#include <Adafruit_BNO08x.h>

// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9

// For SPI mode, we also need a RESET 
#define BNO08X_RESET 5
// but not for I2C or UART
//#define BNO08X_RESET -1

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  setReports();

  Serial.println("Reading events");
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
    Serial.println("Could not enable linear acceleration");
  }
}

float calc3dVec(float x, float y, float z) {
  return sqrt(x*x + y*y + z*z);
}
void loop() {
  delay(10);
  static float highestAccel = 0;
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  
  if (! bno08x.getSensorEvent(&sensorValue)) {
    return;
  }
  

  switch (sensorValue.sensorId) {
    
    case SH2_LINEAR_ACCELERATION:
      Serial.print("Linear Acceration - x: ");
      Serial.print(sensorValue.un.linearAcceleration.x);
      Serial.print(" y: ");
      Serial.print(sensorValue.un.linearAcceleration.y);
      Serial.print(" z: ");
      Serial.println(sensorValue.un.linearAcceleration.z);
      Serial.print("total (converted to g): ");

      float totalAcc = calc3dVec(sensorValue.un.linearAcceleration.x,sensorValue.un.linearAcceleration.y, sensorValue.un.linearAcceleration.z)/9.81;

      Serial.println(totalAcc); 

      if (totalAcc > highestAccel) {
        highestAccel = totalAcc;
      }
      break;
    }

}