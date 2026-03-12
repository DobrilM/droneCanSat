  #include <Adafruit_BMP3XX.h>
  #include <Adafruit_BNO08x.h>


  #define BMP_CS 16
  #define PRESSURE_SEA 1014.3
  Adafruit_BMP3XX bmp;

  //imu define
  #define BNO08X_RESET 18
  Adafruit_BNO08x  bno08x(BNO08X_RESET);
  sh2_SensorValue_t sensorValue;

  float accY;

  uint8_t state = 0;

  float temperature, pressure, altitude;

  unsigned long lastSerial = 0;

  uint8_t navState;
  uint8_t fix;
  unsigned long fixSearchTime = 0;
  unsigned long setNavState = 0;
  //rc send sim
  void setup() {
    Serial.begin(9600);

    if (!bmp.begin_SPI(BMP_CS)) {  
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
      while (1);
    }
    if (!bno08x.begin_I2C()) {
    //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
    //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
      Serial.println("Failed to find BNO08x chip");
      while (1) { delay(10); }
    }
    setReports();
  }

  void setReports(void) {
    Serial.println("Setting desired reports");
    if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
      Serial.println("Could not enable linear acceleration");
    }
  }

  void standbyMode(float h, float a) {
    if (h > 5 ){//&& a > 0.5) {
      state = 1;
    }
  }
  void ascending(float a) {
    if (a < -0.3) {
      state =2;
    }
  }
  void descending(float h, unsigned long now) {
    if (h < 5) {
      fixSearchTime = now;
      state =3;
    }
  }

  void waitForFix(unsigned long now) {
    if (now - fixSearchTime >10000 && fix ==0) {
      state = 5;
    }
    if (fix >0) {
      state = 4;
      setNavState = now;
    }
  }
  void rtwpMode(uint8_t navState) {
    if(navState ==4) {
      state = 5;
    }
  }

  void land() {
    ;
  }
  void loop() {
    unsigned long now = millis();
    if (bmp.performReading()) {
      temperature = bmp.readTemperature();
      pressure = bmp.readPressure() / 100.0;   //hPa
      //altitude =  44330.0 * (1.0 - pow(pressure / PRESSURE_SEA, 0.1903));
      altitude = bmp.readAltitude(PRESSURE_SEA);
    }

    if (bno08x.wasReset()) {
      Serial.print("sensor was reset ");
      setReports();
    }
    
    if (! bno08x.getSensorEvent(&sensorValue)) {
      return;
    }


    switch (sensorValue.sensorId) {  
      case SH2_LINEAR_ACCELERATION:
        accY = sensorValue.un.linearAcceleration.y/9.81f;
        break;
    }

    switch (state) {
        case 0: standbyMode(altitude, accY); break;
        case 1: ascending(accY); break;
        case 2: descending(altitude, now); break;
        case 3: waitForFix(now); break;
        case 4: rtwpMode(navState); break;
        case 5: land(); break;
    }

    if (now - lastSerial> 1000) {
    Serial.print(temperature, 2);
    Serial.print(",");
    Serial.print(pressure, 2);
    Serial.print(",");
    Serial.print(altitude, 2);
    Serial.print(",");
    Serial.print(accY, 3);
    Serial.print(",");
    Serial.print(state);
    Serial.print(",");

      switch (state) {
        case 0: Serial.println("radio off");break;
        case 1: Serial.println("ascending");break;
        case 2: Serial.println("descending");break;
        case 3: Serial.println("waiting for fix");break;
        case 4: Serial.println("going to wp");
        if (now - setNavState > 10000) {
          navState = 4;
        }
        break;
        case 5: Serial.println("landing");break;
      }
      lastSerial =now;
    }
  }