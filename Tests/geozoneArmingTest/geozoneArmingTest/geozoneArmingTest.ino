
#define MSP_INTERVAL 20
#define SERIAL_INTERVAL 1000

//vars
unsigned long lastMSP = 0;
unsigned long lastSerial = 0;
uint16_t rcValues[16];
bool landing = false;

//msp commands
constexpr uint8_t RC_CMD = 200;
constexpr uint8_t GPS_GET = 106;
constexpr uint8_t GYRO_GET = 102;
constexpr uint8_t NAV_STAT = 121;
constexpr uint8_t BATT_GET = 130;

//msp reading var
uint8_t payload[256];  // Increased buffer size
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
float latitude, longitude;
int16_t altitudeGPS;

//gyro reading
float accX, accY, accZ;

//mission state reading
uint8_t navState;

//battery voltage 
float battVolt;

struct point {
  float x;
  float y;
};

constexpr point borders[4] = {
  {52.26943825232155, 4.607911997462024},
  {52.26966098030295, 4.608252639857821},
  
  //{52.26946605905007, 4.608035402425242},
  //{52.26978285972307, 4.608522477454779},
  {52.27000198501937, 4.608016763695906},
  {52.26971329827905, 4.607593366196109},
};

void setup() {
  Serial.begin(115200);
  //while(!Serial);
  Serial.println("MSP INIT:");
  Serial1.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  //msp data array init - 8 channels (standard RC)
  for (int i = 0; i < 16; i++) {
    rcValues[i] = 1500;
  }
  rcValues[2] = 1000;  // Throttle
}

void mspWrite(uint8_t cmd, uint8_t* payload, uint8_t size) {
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

void parsePacket(uint8_t cmd, uint8_t* payload, uint8_t dataSize) {
  switch (cmd) {
    case GPS_GET:
      if (dataSize >= 12) {
        fix = payload[0];
        numSat = payload[1];
        
        // Proper type casting for alignment
        int32_t latRaw = (int32_t)payload[2] | ((int32_t)payload[3] << 8) | ((int32_t)payload[4] << 16) | ((int32_t)payload[5] << 24);
        int32_t lonRaw = (int32_t)payload[6] | ((int32_t)payload[7] << 8) | ((int32_t)payload[8] << 16) | ((int32_t)payload[9] << 24);
        
        latitude = latRaw / 10000000.0f;
        longitude = lonRaw / 10000000.0f;
        altitudeGPS = (int16_t)payload[10] | ((int16_t)payload[11] << 8);
      }
      break;
      
    case NAV_STAT:
      if (dataSize >= 1) {
        navState = payload[0];
      }
      break;
      
    case BATT_GET:
      if (dataSize >= 1) {
        battVolt = payload[0] / 10.0f;
      }
      break;
      
    case GYRO_GET:
      if (dataSize >= 6) {
        // Parse gyro data (usually scaled values)
        accX = (int16_t)payload[0] | ((int16_t)payload[1] << 8);
        accY = (int16_t)payload[2] | ((int16_t)payload[3] << 8);
        accZ = (int16_t)payload[4] | ((int16_t)payload[5] << 8);
      }
      break;
  }

}

void parseMSP(uint8_t readChar) {
  switch (type) {
    case IDLE:
      type = (readChar == '$') ? DOLLAR : IDLE;
      break;
      
    case DOLLAR:
      checksum = 0; 
      type = (readChar == 'M') ? M : IDLE;
      break;
      
    case M:
      type = (readChar == '>') ? ARROW : IDLE;
      break;
      
    case ARROW:
      dataSize = readChar;
      checksum ^= readChar;
      // Prevent buffer overflow
      if (dataSize > 254) {
        type = IDLE;
        return;
      }
      arrayptr = 0;
      type = SIZE;
      break;
      
    case SIZE:
      cmd = readChar;
      checksum ^= readChar;
      if (dataSize == 0) {
        type = CHECKSUM;
      } else {
        type = CMD;
      }
      break; 
      
    case CMD:
      if (arrayptr < dataSize) {
        payload[arrayptr++] = readChar;
        checksum ^= readChar;
      }
      if (arrayptr >= dataSize) {
        type = CHECKSUM;  
      }
      break;

    case CHECKSUM:
      if (checksum == readChar) {
        parsePacket(cmd, payload, dataSize);
      } else {
        // Checksum mismatch - could print debug info
        Serial.print("CSERR ");
      }
      checksum = 0;
      arrayptr = 0;
      type = IDLE;
      break;
  }
}

void mspRead() {
  while (Serial1.available()) {
    parseMSP(Serial1.read());
  }
}

void sendRC() {
  mspWrite(RC_CMD, (uint8_t*)rcValues, 32);
}

void readGPS() {
  mspWrite(GPS_GET, nullptr, 0);
  mspRead();
}

bool vecCheck(point a, point b, point c) {
  return ((b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x) < 0);
}

bool geozoneCheck(point coordinates) {
  uint8_t geoChecksum = 0;
  uint8_t polygonCount = sizeof(borders)/sizeof(borders[0]);
  for (uint8_t i =0; i < polygonCount-1; i++) {
    geoChecksum += vecCheck(borders[i], borders[i+1], coordinates);
    Serial.print(geoChecksum);
  }
  geoChecksum += vecCheck(borders[polygonCount-1], borders[0], coordinates);
  Serial.println(geoChecksum);
  return (geoChecksum == polygonCount);
}

void loop() {
  static bool insideGeozone = false;
  static unsigned long start =millis();
  static unsigned long lastMSP = 0;
  unsigned long now = millis();
  static unsigned long lastGPS = 0;
 

  if (now - lastGPS > 100) {
  readGPS();  
  lastGPS = now;
  Serial.print("Latitude: ");
  Serial.print(latitude, 7);
  Serial.print(';');
  Serial.print("Longitude: ");
  Serial.print(longitude, 7);
  Serial.print(';');
  Serial.print("Fix: ");
  Serial.print(fix);
  Serial.print(';');
  Serial.print("Number of Sats: ");
  Serial.println(numSat);
  point p = {latitude, longitude};  
  //insideGeozone = geozoneCheck(p);
  if (now - start > 10000) {
    
    rcValues[7] = (!insideGeozone) ? 1000 : 2000;
  }
  insideGeozone ? Serial.println("Inside Geozone"): Serial.println("Outside Geozone");

  }

  if (now - lastMSP >50) {
    digitalWrite(LED_BUILTIN, HIGH);
    sendRC();
    lastMSP = now;
    digitalWrite(LED_BUILTIN, LOW);
  }

}