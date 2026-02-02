#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_BMP280.h>
#define RFM95_CS 8
#define RFM95_INT 3
#define RFM95_RST 4
#define RF95_FREQ 433.1

//Note: MSP is connected to Serial1
#define BMP_CS 19
RH_RF95 rf95(RFM95_CS, RFM95_INT);

#define PRESSURE_SEA 1013.25
Adafruit_BMP280 bmp(BMP_CS);

constexpr uint8_t GPS_GET = 106;

int16_t counter = 0;  
struct message {
  uint8_t value;
  int16_t counter;
  int16_t temp;
  int16_t alt; 
  int32_t pressure;
  uint8_t fix;
  uint8_t numSat;
  uint32_t latitude;
  uint32_t longitude;
  int16_t altGPS;
};

uint8_t payload[16];

enum MSPType {
  IDLE,
  DOLLAR,
  M,
  ARROW,
  SIZE,
  CMD,
  PAYLOAD,
  CHECKSUM
};
MSPType type = IDLE;
uint8_t dataSize = 0;
uint8_t cmd = 0;
uint8_t checksum = 0;
uint8_t arrayptr = 0;

//gps reading
uint8_t fix, numSat;
uint32_t latitude, longitude;
int16_t altitudeGPS;

unsigned long lastRadio = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(115200);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  if (!rf95.init()) {
    Serial.println("rf doesnt init");
    while (1);
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("Freq cannot be set");
    while (1);
  }
  rf95.setTxPower(2, false);
    if (!bmp.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

message makeMessage(float temperature, float altitude,  float pressure, uint8_t fix, uint8_t numSat, uint32_t latitude, uint32_t longitude, int16_t altGPS) {
  message p{};
  p.value = 100;
  p.counter = counter++;
  p.temp = temperature * 100; //conv to int
  p.alt = altitude * 100;
  p.pressure = pressure * 100;
  p.fix = fix;
  p.numSat = numSat;
  p.latitude = latitude;
  p.longitude = longitude;
  p.altGPS = altGPS;
  return p;
};


void mspCmd(uint8_t cmd, uint8_t* payload, uint8_t size) {
  uint8_t checksum = 0;
  Serial1.write('$');
  Serial1.write('M');
  Serial1.write('<');
  
  Serial1.write(size);
  checksum ^= size;

  Serial1.write(cmd);
  checksum ^= cmd;

  for (uint8_t i = 0; i < size; i++) {
      Serial1.write(payload[i]);
      checksum ^= payload[i];
  }
  Serial1.write(checksum);
}

void parsePacket(uint8_t cmd) {
    switch (cmd) {
    case GPS_GET:
    fix  = payload[0];
    numSat = payload[1];

    latitude = *(uint32_t*)&payload[2];
    longitude = *(uint32_t*)&payload[6];
    altitudeGPS = (*(int16_t*)&payload[10]);
    break;


  }
}

void parseMSP(uint8_t readChar) {
  switch (type) {
  case IDLE: type = (readChar == '$') ? DOLLAR : IDLE; break;
  case DOLLAR: type = (readChar == 'M') ? M : IDLE; break;
  case M: type = (readChar == '>') ? ARROW : IDLE; break;
  case ARROW: dataSize = readChar; checksum ^= readChar; type = SIZE; break;
  case SIZE: cmd = readChar; checksum ^= readChar; type = CMD; arrayptr = 0; break;
  case CMD: 
    if (arrayptr < dataSize) {
      payload[arrayptr] = readChar;
      arrayptr++;
      checksum ^= readChar;
    } else {
      type = CHECKSUM;
    }
  break;
  case CHECKSUM: 
    if (checksum == readChar) {
      parsePacket(cmd);
    }
    checksum = 0;
    type = IDLE;
    break;

  }
}

void mspReadGPS() {
  mspCmd(GPS_GET, nullptr, 0);
  while (Serial1.available()) {
    parseMSP(Serial1.read());
  }
}

void loop() {
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0;   //hPa
  float altitude =  44330.0 * (1.0 - pow(pressure / PRESSURE_SEA, 0.1903));
  mspReadGPS();
  unsigned long now = millis();
  if (now - lastRadio >= 1000) {
    digitalWrite(LED_BUILTIN, HIGH);
    message pkt = makeMessage(temperature, altitude, pressure, fix, numSat, latitude, longitude, altitudeGPS);
    rf95.send((uint8_t*)&pkt, sizeof(pkt));
    rf95.waitPacketSent();
    Serial.println("Message sent!");
    digitalWrite(LED_BUILTIN, LOW);
    lastRadio = now;
    Serial.print(temperature);
    Serial.print(pressure);
    Serial.println(altitude);
    Serial.print(fix);
    Serial.println(numSat);
    Serial.print(latitude);
    Serial.println(longitude);
    Serial.println(altitudeGPS);
  }
}
