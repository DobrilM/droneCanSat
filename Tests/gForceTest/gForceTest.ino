// Basic demo for readings from Adafruit BNO08x
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <RH_RF95.h>
// For SPI mode, we need a CS pin
//#define BNO08X_CS 10
//#define BNO08X_INT 9

// For SPI mode, we also need a RESET 
//#define BNO08X_RESET 5
// but not for I2C or UART

#define BNO08X_RESET 18
#define RFM95_CS 8
#define RFM95_INT 3
#define RFM95_RST 4
#define RF95_FREQ 433.4
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
float accX, accY, accZ, highestAccel, totalAcc, absoluteAcc;
RH_RF95 rf95(RFM95_CS, RFM95_INT);

struct message {
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  int16_t accTot;
  int16_t accMax;
};

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x test!");

  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  setReports();

  Serial.println("Reading events");
  delay(100);
    // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  if (!rf95.init()) {
    Serial.println("rf doesnt init");
    while (1)
      ;
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("Freq cannot be set");
    while (1)
      ;
  }
  rf95.setTxPower(2, false);
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

message makeMessage() {
  message p{};
  p.accX = accX *100.0;
  p.accY = accY*100.0;
  p.accZ = accZ*100.0;
  p.accTot = absoluteAcc*100.0;
  p.accMax = highestAccel*100.0;
  return p;
};


void loop() {
  delay(10);
  unsigned long now = millis();
  static unsigned long lastRadio = 0;
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  
  if (! bno08x.getSensorEvent(&sensorValue)) {
    return;
  }
  
  switch (sensorValue.sensorId) {
    
    case SH2_LINEAR_ACCELERATION:
      accX = sensorValue.un.linearAcceleration.x/9.81;
      accY = sensorValue.un.linearAcceleration.y/9.81;
      accZ = sensorValue.un.linearAcceleration.z/9.81;
      Serial.print("Linear Acceration - x: ");
      Serial.print(accX);
      Serial.print(" y: ");
      Serial.print(accY);
      Serial.print(" z: ");
      Serial.println(accZ);
      Serial.print("total (converted to g): ");

      totalAcc = calc3dVec(accX,accY,accZ);
      absoluteAcc = fabs(totalAcc);
      Serial.print(totalAcc);
      Serial.print(" total (converted to g, absolute): ");
      Serial.println(absoluteAcc); 

      if (absoluteAcc > highestAccel) {
        highestAccel = absoluteAcc;
      }
      Serial.print("highest acceleration (absolute)");
      Serial.println(highestAccel);
      break;
    }
  if (now - lastRadio >= 1000) {
    digitalWrite(LED_BUILTIN, HIGH);
    message pkt = makeMessage();
    rf95.send((uint8_t*)&pkt, sizeof(pkt));
    rf95.waitPacketSent();
    Serial.println("Message sent!");
    digitalWrite(LED_BUILTIN, LOW);
    lastRadio = now;
  }
}
