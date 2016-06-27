/*
 * 4x4 board
 * by JJurM
 */

//===== CONSTANTS =====

#define SERIAL_SPEED 115200

//----- LEDS -----

#define LED_HEARTBEAT 0
#define LED_TILT_ALERT 4
#define LED_PWR 6
#define LED_ERROR 7

//----- LIGHT SYSTEM -----

// interval in ms between light check
#define LIGHT_SYSTEM_CHECK_INTERVAL 50

// how long to look into past
#define LIGHT_SYSTEM_TRESHOLD_TIME 1000

// ratio (positive checks / all checks) required to fire tilt alert
#define LIGHT_SYSTEM_TRESHOLD_RATIO 0.9

// minimal duration of tilt alert
#define LIGHTS_ON_DURATION 2000

//----- TILT ALERT -----

#define TILT_ALERT_TRESHOLD_ANGLE 25.0

// interval in ms between tilt check
#define TILT_ALERT_CHECK_INTERVAL 50

// how long to look into past
#define TILT_ALERT_TRESHOLD_TIME 600

// ratio (positive checks / all checks) required to fire tilt alert
#define TILT_ALERT_TRESHOLD_RATIO 0.9

// minimal duration of tilt alert
#define TILT_ALERT_DURATION 500

//----- COMPUTED CONSTANTS -----
#define LIGHT_SYSTEM_CHECK_COUNT (LIGHT_SYSTEM_TRESHOLD_TIME / LIGHT_SYSTEM_CHECK_INTERVAL)
#define LIGHT_SYSTEM_TRESHOLD_COUNT ((int)(LIGHT_SYSTEM_CHECK_COUNT * LIGHT_SYSTEM_TRESHOLD_RATIO))

#define TILT_ALERT_CHECK_COUNT (TILT_ALERT_TRESHOLD_TIME / TILT_ALERT_CHECK_INTERVAL)
#define TILT_ALERT_TRESHOLD_COUNT ((int)(TILT_ALERT_CHECK_COUNT * TILT_ALERT_TRESHOLD_RATIO))

//----- OTHER -----

#define EEPROM_ADDR_STARTEDCOUNT 0
#define EEPROM_ADDR_ERRORINDEX 2

#define I2C_ADDR_MPU6050 0x68

//===== PINOUT =====

#define PIN_LIGHT_SENSOR1 A5
#define PIN_LIGHT_SENSOR2 A4
#define PIN_LIGHT_TRESHOLD A3

#define PIN_LIGHTS1 11
#define PIN_LIGHTS2 10
#define PIN_LIGHTS3 9
#define PIN_LIGHTS4 6

#define PIN_BTN1 2
#define PIN_BTN2 3

/*#define PIN_SDA 14
#define PIN_SCL 15*/
#define SDA_PORT PORTC
#define SDA_PIN 1
#define SCL_PORT PORTC
#define SCL_PIN 0

#define PIN_SWITCH1 5
#define PIN_SWITCH2 4
#define PIN_SWITCH3 7
#define PIN_SWITCH4 8

#define PIN_SR_SER 12
#define PIN_SR_SCK 13
#define PIN_SR_RCK 16

//===== INCLUDES =====
#include <SoftI2CMaster.h>
#include <EEPROM.h>

//===== VARIABLES =====
int eepromErrorIndex;
byte leds;

boolean lightSystemChecks[LIGHT_SYSTEM_CHECK_COUNT];
int lightSystemChecksIndex = 0;
long lightSystemLastCheck = 0;
long lightsOnLastFired = 0;
boolean lightsState = false;

boolean tiltAlertChecks[TILT_ALERT_CHECK_COUNT];
int tiltAlertChecksIndex = 0;
long tiltAlertLastCheck = 0;
long tiltAlertLastFired = 0;
boolean tiltAlertState = false;

int16_t AcX,AcY,AcZ;
double angle;

//----- temporary -----
boolean result;
int count;
long now;

//===== UTILS =====
void writeLeds() {
  digitalWrite(PIN_SR_RCK, HIGH);
  shiftOut(PIN_SR_SER, PIN_SR_SCK, MSBFIRST, leds);
  digitalWrite(PIN_SR_RCK, LOW);
}

void setLed(int index, boolean value) {
  leds ^= (-value ^ leds) & (1 << index);
}

void writeLed(int index, boolean value) {
  setLed(index, value);
  writeLeds();
}

int readTreshold() {
  return analogRead(PIN_LIGHT_TRESHOLD);
}

boolean checkLight() {
  //+ todo
}

void readAccelValues() {
  i2c_start((I2C_ADDR_MPU6050 << 1) | I2C_WRITE);
  i2c_write(0x3B);
  i2c_stop();
  
  i2c_start((I2C_ADDR_MPU6050 << 1) | I2C_READ);
  //i2c_rep_start((I2C_ADDR_MPU6050 << 1) | I2C_READ);
  AcX=i2c_read(false)<<8|i2c_read(false);  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=i2c_read(false)<<8|i2c_read(false);  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=i2c_read(false)<<8|i2c_read(true);   // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  i2c_stop();
}

double computeAngle(int32_t x, int32_t y, int32_t z) {
  return 90 - (acos( (x*x + y*y) / (sqrt(sq(x) + sq(y) + sq(z)) * sqrt(sq(x) + sq(y))) ) * 57.2958);
}

boolean checkTilt() {
  readAccelValues();
  angle = computeAngle(AcX, AcY, AcZ);
  return angle >= TILT_ALERT_TRESHOLD_ANGLE;
}

int getEepromErrorIndex() {
  return (EEPROM.read(EEPROM_ADDR_ERRORINDEX) << 8) | EEPROM.read(EEPROM_ADDR_ERRORINDEX + 1);
}

void writeEepromErrorIndex() {
  EEPROM.update(EEPROM_ADDR_ERRORINDEX, (eepromErrorIndex >> 8) & 0xFF);
  EEPROM.update(EEPROM_ADDR_ERRORINDEX + 1, eepromErrorIndex & 0xFF);
}

void writeError(byte num) {
  /**
   * List of errors:
   *  1 - test error
   *  2 - I2C: Initialization error. SDA or SCL are low
   *  3 - I2C: device MPU6050 not found
   */

  writeLed(LED_ERROR, HIGH);
  
  EEPROM.update(eepromErrorIndex, num);
  eepromErrorIndex++;
  writeEepromErrorIndex();
}

//===== PROGRAM =====

void pLightSystem() {
  now = millis();
  if (now >= lightSystemLastCheck + LIGHT_SYSTEM_CHECK_INTERVAL) {
    lightSystemLastCheck = now;
    
    result = checkTilt();
    lightSystemChecksIndex = (lightSystemChecksIndex + 1) % LIGHT_SYSTEM_CHECK_COUNT;
    lightSystemChecks[lightSystemChecksIndex] = result;
  }

  count = 0;
  for (int i = 0; i < LIGHT_SYSTEM_CHECK_COUNT; i++) {
    if (lightSystemChecks[i]) count++;
  }
  if (count >= LIGHT_SYSTEM_TRESHOLD_COUNT) {
    lightsOnLastFired = now;
  }

  if ((now <= lightsOnLastFired + LIGHTS_ON_DURATION) != lightsState) {
    lightsState ^= true;
    //+ turn lights on/off
  }
}

void pTiltAlert() {
  now = millis();
  if (now >= tiltAlertLastCheck + TILT_ALERT_CHECK_INTERVAL) {
    tiltAlertLastCheck = now;
    
    result = checkTilt();
    writeLed(5, result);
    tiltAlertChecksIndex = (tiltAlertChecksIndex + 1) % TILT_ALERT_CHECK_COUNT;
    tiltAlertChecks[tiltAlertChecksIndex] = result;

    count = 0;
    for (int i = 0; i < TILT_ALERT_CHECK_COUNT; i++) {
      if (tiltAlertChecks[i]) count++;
    }
    if (count >= TILT_ALERT_TRESHOLD_COUNT) {
      tiltAlertLastFired = now;
    }
  
    if ((now <= tiltAlertLastFired + TILT_ALERT_DURATION) != tiltAlertState) {
      tiltAlertState ^= true;
  
      writeLed(LED_TILT_ALERT, tiltAlertState);
    }
  }
}

void pButtonTest() {
  setLed(0, !digitalRead(PIN_SWITCH1));
  setLed(1, !digitalRead(PIN_SWITCH2));
  setLed(2, !digitalRead(PIN_SWITCH3));
  setLed(3, !digitalRead(PIN_SWITCH4));

  digitalWrite(PIN_LIGHTS1, digitalRead(PIN_SWITCH1));
  digitalWrite(PIN_LIGHTS2, digitalRead(PIN_SWITCH2));
  digitalWrite(PIN_LIGHTS3, digitalRead(PIN_SWITCH3));
  digitalWrite(PIN_LIGHTS4, digitalRead(PIN_SWITCH4));

  setLed(4, !digitalRead(PIN_BTN1));
  setLed(5, !digitalRead(PIN_BTN2));
  
  setLed(LED_HEARTBEAT, (millis()/500) % 2);
  writeLeds();
}

void pEraseEeprom() {
  for ( int i = 0 ; i < EEPROM.length() ; i++ ) {
    EEPROM.write(i, 0);
  }
}

void pSerial() {
  if (Serial.available() > 0) {
    char ch = Serial.read();
    switch (ch) {
      case '.': //*** test
        Serial.println(".");
        break;
      case 'r': //*** read eeprom
        for (int i = 0; i < 16; i++) {
          Serial.print(EEPROM.read(i));
          Serial.print(",");
        }
        Serial.println();
        break;
      case 'n': //*** get eepromErrorIndex - 16 ( = number of errors )
        Serial.println(getEepromErrorIndex() - 16);
        break;
      case 'f': //*** erase first 16 bytes of eeprom
        for (int i = 0; i < 16; i++) {
          EEPROM.update(i, 0);
        }
        Serial.println(".");
        break;
      case 'e': //*** read errors
        for (int i = 16; i < eepromErrorIndex; i++) {
          Serial.print(EEPROM.read(i));
          Serial.print(",");
        }
        Serial.println();
        break;
      case 'p': //*** clear errors
        eepromErrorIndex = 16;
        writeEepromErrorIndex();
        writeLed(LED_ERROR, LOW);
        Serial.println(".");
        break;
      case 'c': //*** throw test error
        writeError(1);
        Serial.println(".");
        break;
      case 'x': //*** chip reset
        asm volatile ("  jmp 0");
        break;
      case 't': //*** read light treshold
        Serial.println(readTreshold());
        break;
    }
  }
}

void pAccelTest() {
  while (true) {
    readAccelValues();
    
    Serial.print("AcX = "); Serial.print(AcX);
    Serial.print(" | AcY = "); Serial.print(AcY);
    Serial.print(" | AcZ = "); Serial.print(AcZ);
    Serial.print(" | Angle = "); Serial.println(computeAngle(AcX, AcY, AcZ));
    delay(300);
  }
}

void setup() {
  // begin serial
  Serial.begin(SERIAL_SPEED);

  //Serial.println(TILT_ALERT_CHECK_COUNT);

  // update startedCount
  int startedCount = (EEPROM.read(EEPROM_ADDR_STARTEDCOUNT) << 8) | EEPROM.read(EEPROM_ADDR_STARTEDCOUNT + 1);
  if (startedCount == ((1 << 16) - 1)) {
    startedCount = 0;
  }
  startedCount++;
  Serial.print("counter: ");
  Serial.println(startedCount);
  EEPROM.update(EEPROM_ADDR_STARTEDCOUNT, (startedCount >> 8) & 0xFF);
  EEPROM.update(EEPROM_ADDR_STARTEDCOUNT + 1, startedCount & 0xFF);

  // retreive eepromErrorIndex
  eepromErrorIndex = getEepromErrorIndex();
  if (eepromErrorIndex == ((1 << 16) - 1) || eepromErrorIndex < 16) {
    eepromErrorIndex = 16;
    writeEepromErrorIndex();
  }
  Serial.print("error index: ");
  Serial.println(eepromErrorIndex);
  
  // set up pins
  pinMode(PIN_SR_SER, OUTPUT);
  pinMode(PIN_SR_SCK, OUTPUT);
  pinMode(PIN_SR_RCK, OUTPUT);

  pinMode(PIN_SWITCH1, INPUT);
  pinMode(PIN_SWITCH2, INPUT);
  pinMode(PIN_SWITCH3, INPUT);
  pinMode(PIN_SWITCH4, INPUT);
  digitalWrite(PIN_SWITCH1, HIGH);
  digitalWrite(PIN_SWITCH2, HIGH);
  digitalWrite(PIN_SWITCH3, HIGH);
  digitalWrite(PIN_SWITCH4, HIGH);

  // light power led
  writeLed(LED_PWR, HIGH);

  // initialize I2C
  if (!i2c_init()) {
    writeError(2);
  }
  // setup MPU6050
  result = i2c_start((I2C_ADDR_MPU6050 << 1) | I2C_WRITE);
  if (result) {
    i2c_write(0x6B); // PWR_MGMT_1 register
    i2c_write(0);    // set to zero (wakes up the MPU-6050)  
  }
  i2c_stop();
  if (!result) {
    writeError(3);
  }

}

void loop() {
  /*pLightSystem();*/
  pTiltAlert();
  pSerial();

  //pButtonTest();
}
