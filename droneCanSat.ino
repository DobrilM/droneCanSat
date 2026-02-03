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
constexpr uint8_t GYRO_GET = 102;
constexpr uint8_t NAV_STAT = 121;
constexpr uint8_t BATT_GET = 130;
constexpr uint8_t RC_CMD = 200;

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
  uint16_t battVolt;
  uint8_t navStat;
  uint8_t status;
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
int16_t altGPS;

float accX, accY, accZ;

uint16_t battVolt;
unsigned long lastRadio = 0;
unsigned long lastMSP;
uint8_t navStat;

uint16_t rcValues[16];

int status = 0;
bool landing = 0;
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

  for (int i = 0; i < 16; i++) {
    rcValues[i] = 1500;
  }
  rcValues[2] = 1000; //throttle
}

message makeMessage(float temperature, float altitude, float pressure) {
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
  p.battVolt = battVolt;
  p.navStat = navStat;
  p.status = status;

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
    altGPS = (*(int16_t*)&payload[10]);
    break;
    case GYRO_GET: //accel wont be sent, so thats why the convertion is made (analog acceleration -> g-force -> acceleration (m/s^2))
    accX  = *(int16_t*)&payload[6]/1670.13;
    accY  = *(int16_t*)&payload[8]/1670.13;
    accZ  = *(int16_t*)&payload[10]/1670.13; 
    break;
    case BATT_GET:
    battVolt = *(uint16_t*)&payload[0];
    break;
    case NAV_STAT:
    navStat = payload[0];
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

void mspReadGyro() {
  mspCmd(GYRO_GET, nullptr, 0);
  while (Serial1.available()) {
    parseMSP(Serial1.read());
  }
}

void mspReadVoltage() {
  mspCmd(BATT_GET, nullptr, 0);
  while (Serial1.available()) {
    parseMSP(Serial1.read());
  }
}

void mspReadMissionStatus() {
  mspCmd(NAV_STAT, nullptr, 0);
  while (Serial1.available()) {
    parseMSP(Serial1.read());
  }
}

void standbyMode(float h, float a) {
  if (h > 50 && a > 3) { //3 m/s^2
      status = 1;
  }
}

void launchedMode(float h, float a) {
  if (h < 300 && a < 0 ) {
    rcValues[4] = 2000; //arm the fc 
    status = 2;
  }
}

void rtwpMode() {
  rcValues[5] = 2000; //ch 6, set to navigate mission
  if (navStat == 0) {
    status = 3;
  }
}
 
void loop() {
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0;   //hPa
  float altitude =  44330.0 * (1.0 - pow(pressure / PRESSURE_SEA, 0.1903));
  mspReadGPS();
  mspReadGyro();
  mspReadVoltage();
  mspReadMissionStatus();
  unsigned long now = millis();

  switch (status) {
    case 0: standbyMode(altitude, accY); break;
    case 1: launchedMode(altitude, accY); break;
    case 2: rtwpMode(); break;
    case 3: landing = true; break;
    default: 
    while(1) {
      delay(100);
    }
    break;
  }

  if (now - lastMSP >= 20 && !landing) {
    mspCmd(RC_CMD, (uint8_t*)rcValues, 32);
    lastMSP = now;
  }

  if (now - lastRadio >= 1000) {
    digitalWrite(LED_BUILTIN, HIGH);
    message pkt = makeMessage(temperature, altitude, pressure);
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
    Serial.println(altGPS);
    Serial.print(accX);
    Serial.print(accY);
    Serial.println(accZ);
    Serial.println(battVolt);
    Serial.println(navStat);
    Serial.println(status);
  }
}
