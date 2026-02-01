#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_BMP280.h>
#define RFM95_CS 8
#define RFM95_INT 3
#define RFM95_RST 4
#define RF95_FREQ 433.1

#define BMP_CS 19
RH_RF95 rf95(RFM95_CS, RFM95_INT);

Adafruit_BMP280 bmp();

int16_t i = 0;
struct message {
  uint8_t value;
  int16_t counter;
  int16_t temp;
  int16_t alt; 
  int16_t pressure;
};

message makeMessage(float temperature, float altitude,  float pressure) {
  message p{};
  p.value = 100;
  p.counter = i++;
  p.temp = temperature * 100; //conv to int
  p.alt = altitude * 100;
  p.pressure = pressure * 100;
  return p;
};

unsigned long lastRadio = 0;
void setup() {
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
  if (!bmp.init(BMP_CS)) {
    Serial.print(No BMP found);
  }
}


void loop() {
  float temperature = bmp.temperature();
  float pressure = bmp.pressure / 100.0;   //hPa
  float altitude =  44330.0 * (1.0 - pow(pressure / PRESSURE_SEA, 0.1903));

  unsigned long now = millis();
  if (now - lastRadio >= 1000) {
    digitalWrite(LED_BUILTIN, HIGH);
    message pkt = makeMessage(temperature, altitude, pressure);
    rf95.send((uint8_t*)&pkt, sizeof(pkt));
    rf95.waitPacketSent();
    Serial.println("Message sent!");
    digitalWrite(LED_BUILTIN, LOW);
    lastRadio = now;
  }
}
