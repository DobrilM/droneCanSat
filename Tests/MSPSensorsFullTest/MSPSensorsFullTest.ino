#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_BMP3XX.h>

#define RFM95_CS 8
#define RFM95_INT 3
#define RFM95_RST 4
#define RF95_FREQ 433.4 //CHANGE BEFORE LAUNCH!!!!!
RH_RF95 rf95(RFM95_CS, RFM95_INT);


//msp command codes
constexpr uint8_t GPS_GET = 106;
constexpr uint8_t GYRO_GET = 102;
constexpr uint8_t NAV_STAT = 121;
constexpr uint8_t BATT_GET = 130;
constexpr uint8_t RC_CMD = 200;

unsigned long lastMSP = 0;
//type of character read by msp
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

//structure of the byte array for the message sent through radio
struct message {
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
  int16_t accX;
  int16_t accY;
  int16_t accZ;
};    

#define BMP_CS 16
#define PRESSURE_SEA 982.3 //CHANGE BEFORE LAUNCH!!!!!
Adafruit_BMP3XX bmp;

//byte array for reading msp
uint8_t payload[16];

//other variables needed for reading msp
MSPType type = IDLE;
uint8_t dataSize = 0;
uint8_t cmd = 0;
uint8_t checksum = 0;
uint8_t arrayptr = 0;

float pressure, temperature, altitude;
//gps reading
uint8_t fix, numSat;
uint32_t latitude, longitude;
int16_t altGPS;

//gyro reading
int16_t accX, accY, accZ;

//battery reading
uint16_t battVolt;

//reading mission status
uint8_t navStat;
unsigned long turnOnTime = millis();
unsigned long lastSerial = 0;

//values representing all channels for simulating rc input to fc
uint16_t rcValues[16];


//setup function
void setup() {
  //Serial 0 init (only used for the initializing of the arduino)
  Serial.begin(9600);
    while (!Serial);   // <-- IMPORTANT on M0
    //rf95 setup
  Serial.print("read");
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  Serial.println("rfm95 pin reset");
  if (!rf95.init()) {
    Serial.println("rf doesnt init");
    while (1);
  }
  Serial.println("rfm95 freq");
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("Freq cannot be set");
    while (1);
  }
  rf95.setTxPower(23, false);

  //bmp init
  if (!bmp.begin_SPI(BMP_CS)) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }

  //msp init
  Serial1.begin(115200);

  //initializing values for the rc controlling of the fc 
  for (int i = 0; i < 16; i++) {
    rcValues[i] = 1500;
  }
  rcValues[2] = 1000; //throttle needs to be low for arming
  
}


//assigning all data to byte array, ready to be sent through radio
message makeMessage(float temperature, float altitude, float pressure) {
  message p{};
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
  p.accX = accX;
  p.accY = accY;
  p.accZ = accZ;

  return p;
};

//writing command to fc using msp
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

//parsing the retrieved packet, using the read command as a baseline
void parsePacket(uint8_t cmd) {
  switch (cmd) {

    case GPS_GET:
      fix  = payload[0];
      numSat = payload[1];
      latitude  = *(uint32_t*)&payload[2];
      longitude = *(uint32_t*)&payload[6];
      altGPS    = *(int16_t*)&payload[10];
      break;

    case GYRO_GET:
      accX = *(int16_t*)&payload[0];
      accY = *(int16_t*)&payload[2];
      accZ = *(int16_t*)&payload[4];
      break;

    case BATT_GET:
      battVolt = payload[0];
      break;

    case NAV_STAT:
      navStat = payload[0];
      break;
  }
}

//parsing each byte one by one and creating a packet of data
void parseMSP(uint8_t readChar) {
  switch (type) {
  case IDLE: type = (readChar == '$') ? DOLLAR : IDLE; break;
  case DOLLAR:
  type = (readChar == 'M') ? M : IDLE; break;
  case M:
  type = (readChar == '>') ? ARROW : IDLE; break;
  case ARROW:
  dataSize = readChar; type = SIZE; break;
  case SIZE: 
  cmd = readChar; checksum ^= readChar; type = CMD; arrayptr = 0; break;
  case CMD: 
    if (arrayptr < dataSize) {
      payload[arrayptr++] = readChar;
      checksum ^= readChar;
    } else {
      type = CHECKSUM;
    }
  break;
  case CHECKSUM: 
    parsePacket(cmd);
    checksum = 0;
    type = IDLE;
    break;

  }
}

//reading gps
void mspReadBatt() {
  mspCmd(BATT_GET, nullptr, 0);
  while (Serial1.available()) {
    parseMSP(Serial1.read());
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

void mspReadMission() {
  mspCmd(NAV_STAT, nullptr, 0);
  while (Serial1.available()) {
    parseMSP(Serial1.read());
  }
}

//main loop
void loop() {
  static unsigned long lastTelemetry = 0;
    static unsigned long lastBMP = 0;
    unsigned long now = millis();
  if (now - lastTelemetry > 500) {
    mspReadBatt();
    mspReadGPS();
    mspReadGyro();
    mspReadMission();

    lastTelemetry = now;
  }

  //if (now - lastBMP > 100) {
    //bmp.performReading();
    //lastBMP = now;
  //}
  //sending msp`
  if (now - lastMSP >= 20) {
    mspCmd(RC_CMD, (uint8_t*)rcValues, 32);
    lastMSP = now;
  }
  if (bmp.performReading()) {
    temperature = bmp.temperature;
    pressure = bmp.pressure / 100.0;   //hPa
    altitude =  bmp.readAltitude(PRESSURE_SEA);
  }
  if (now - lastSerial >= 1000) {
    digitalWrite(LED_BUILTIN, HIGH);
    message pkt = makeMessage(temperature, altitude, pressure);
    rf95.send((uint8_t*)&pkt, sizeof(pkt));
    rf95.waitPacketSent();
    digitalWrite(LED_BUILTIN, LOW);
    lastSerial = now;
    Serial.println("packet sent");
  } 


}
