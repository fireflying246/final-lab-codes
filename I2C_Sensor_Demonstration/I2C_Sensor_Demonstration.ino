#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_BME280.h>

/*
 * Robot I2C sensor demonstration.
 * ADS1115 reads three IR sensors, BME280 reads environmental data,
 * and the MPU is accessed through raw I2C registers.
 * A dual-threshold Z-axis latch controls the motors.
 */
const int enablePinRight = 5;
const int enablePinLeft  = 6;

const int in1Pin = 4;
const int in2Pin = 7;

const int in3Pin = 9;
const int in4Pin = 10;

// LED
const int eyeLedPin = 3;
const int statusLedPin = 11;

const int IR_LEFT_CH   = 0;
const int IR_CENTER_CH = 1;
const int IR_RIGHT_CH  = 2;

const int ADS1115_ADDR = 0x48;
const int BME280_ADDR1 = 0x76;
const int BME280_ADDR2 = 0x77;
const int MPU_ADDR     = 0x68;

const byte MPU_REG_SMPLRT_DIV    = 0x19;
const byte MPU_REG_CONFIG        = 0x1A;
const byte MPU_REG_GYRO_CONFIG   = 0x1B;
const byte MPU_REG_ACCEL_CONFIG  = 0x1C;
const byte MPU_REG_PWR_MGMT_1    = 0x6B;
const byte MPU_REG_PWR_MGMT_2    = 0x6C;
const byte MPU_REG_WHO_AM_I      = 0x75;

const byte MPU_REG_ACCEL_XOUT_H = 0x3B;
const byte MPU_REG_GYRO_XOUT_H  = 0x43;

bool reverseRightMotor = false;
bool reverseLeftMotor  = false;

const int testForwardSpeed = 150;

const int IR_BLACK_THRESHOLD = 10000;

const float MPU_ACCEL_SCALE = 16384.0;

const float MPU_GYRO_SCALE = 131.0;

const float STANDARD_GRAVITY = 9.80665;

const float GYRO_OFFSET_X = 14.35;
const float GYRO_OFFSET_Y = 1.05;
const float GYRO_OFFSET_Z = 0.30;

const float LIFT_TRIGGER = -8.0;
const float DROP_STOP = -15.0;

const bool INVERT_Z_FOR_CONTROL = true;

const unsigned long RECORD_INTERVAL_MS = 100;

const unsigned long NORMAL_PRINT_INTERVAL_MS = 1200;

const unsigned long FAST_PRINT_INTERVAL_MS = 800;

const unsigned long FAST_PRINT_DURATION_MS = 2000;

const unsigned long MOTION_INTERVAL_MS = 30;

const float HUMIDITY_CHANGE_TRIGGER = 1.0;

const float GYRO_CHANGE_TRIGGER = 2.0;

Adafruit_ADS1115 ads;
Adafruit_BME280 bme;

bool adsOK = false;
bool bmeOK = false;
bool mpuOK = false;

byte mpuWhoAmI = 0x00;

bool isRunning = false;

float controlZRawMps2 = 0.0;

float controlZUsedMps2 = 0.0;

bool controlMPUReadOK = false;

int16_t recordIrLeftRaw = 0;
int16_t recordIrCenterRaw = 0;
int16_t recordIrRightRaw = 0;

bool recordIrLeftBlack = false;
bool recordIrCenterBlack = false;
bool recordIrRightBlack = false;

float recordTemperatureC = 0.0;
float recordPressureHpa = 0.0;
float recordHumidity = 0.0;

float recordAx = 0.0;
float recordAy = 0.0;
float recordAz = 0.0;

float recordGx = 0.0;
float recordGy = 0.0;
float recordGz = 0.0;

bool recordMPUReadOK = false;

bool humidityReferenceReady = false;
float lastHumidityForTrigger = 0.0;

bool gyroReferenceReady = false;
float lastGxForTrigger = 0.0;
float lastGyForTrigger = 0.0;
float lastGzForTrigger = 0.0;

unsigned long fastPrintUntilTime = 0;

unsigned long lastRecordTime = 0;
unsigned long lastPrintTime = 0;
unsigned long lastMotionTime = 0;

void setSingleMotor(
  int enablePin,
  int dirPin1,
  int dirPin2,
  int speedValue,
  bool reverseMotor
)
{
  speedValue = constrain(speedValue, -255, 255);

  if (reverseMotor) {
    speedValue = -speedValue;
  }

  if (speedValue > 0) {
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, LOW);
    analogWrite(enablePin, speedValue);
  }
  else if (speedValue < 0) {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, HIGH);
    analogWrite(enablePin, -speedValue);
  }
  else {
    digitalWrite(dirPin1, LOW);
    digitalWrite(dirPin2, LOW);
    analogWrite(enablePin, 0);
  }
}

void setMotorSpeed(int leftSpeed, int rightSpeed)
{
  setSingleMotor(
    enablePinLeft,
    in3Pin,
    in4Pin,
    leftSpeed,
    reverseLeftMotor
  );

  setSingleMotor(
    enablePinRight,
    in1Pin,
    in2Pin,
    rightSpeed,
    reverseRightMotor
  );
}

void stopMotors()
{
  setMotorSpeed(0, 0);
}

void runForwardForExperiment()
{
  setMotorSpeed(testForwardSpeed, testForwardSpeed);
}

bool writeMPUByte(byte reg, byte data)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);

  byte error = Wire.endTransmission();

  return error == 0;
}

byte readMPUByte(byte reg)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  byte error = Wire.endTransmission(false);

  if (error != 0) {
    return 0xFF;
  }

  byte count = Wire.requestFrom(MPU_ADDR, 1);

  if (count == 1 && Wire.available()) {
    return Wire.read();
  }

  return 0xFF;
}

bool readMPUWord(byte reg, int16_t &value)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);

  byte error = Wire.endTransmission(false);

  if (error != 0) {
    value = 0;
    return false;
  }

  byte count = Wire.requestFrom(MPU_ADDR, 2);

  if (count == 2 && Wire.available() >= 2) {
    int16_t highByte = Wire.read();
    int16_t lowByte = Wire.read();

    value = (highByte << 8) | lowByte;

    return true;
  }

  value = 0;
  return false;
}

bool readMPURawData(
  float &ax,
  float &ay,
  float &az,
  float &gx,
  float &gy,
  float &gz
)
{
  int16_t axRaw = 0;
  int16_t ayRaw = 0;
  int16_t azRaw = 0;

  int16_t gxRaw = 0;
  int16_t gyRaw = 0;
  int16_t gzRaw = 0;

  bool ok = true;

  ok = ok && readMPUWord(
    MPU_REG_ACCEL_XOUT_H,
    axRaw
  );

  ok = ok && readMPUWord(
    MPU_REG_ACCEL_XOUT_H + 2,
    ayRaw
  );

  ok = ok && readMPUWord(
    MPU_REG_ACCEL_XOUT_H + 4,
    azRaw
  );

  ok = ok && readMPUWord(
    MPU_REG_GYRO_XOUT_H,
    gxRaw
  );

  ok = ok && readMPUWord(
    MPU_REG_GYRO_XOUT_H + 2,
    gyRaw
  );

  ok = ok && readMPUWord(
    MPU_REG_GYRO_XOUT_H + 4,
    gzRaw
  );

  if (!ok) {
    ax = 0.0;
    ay = 0.0;
    az = 0.0;

    gx = 0.0;
    gy = 0.0;
    gz = 0.0;

    return false;
  }

  if (
    axRaw == 0 &&
    ayRaw == 0 &&
    azRaw == 0 &&
    gxRaw == 0 &&
    gyRaw == 0 &&
    gzRaw == 0
  ) {
    ax = 0.0;
    ay = 0.0;
    az = 0.0;

    gx = 0.0;
    gy = 0.0;
    gz = 0.0;

    return false;
  }

  ax = axRaw / MPU_ACCEL_SCALE;
  ay = ayRaw / MPU_ACCEL_SCALE;
  az = azRaw / MPU_ACCEL_SCALE;

  gx = gxRaw / MPU_GYRO_SCALE;
  gy = gyRaw / MPU_GYRO_SCALE;
  gz = gzRaw / MPU_GYRO_SCALE;

  return true;
}

bool readMPUAccelZ(float &az)
{
  int16_t azRaw = 0;

  bool ok = readMPUWord(
    MPU_REG_ACCEL_XOUT_H + 4,
    azRaw
  );

  if (!ok) {
    az = 0.0;
    return false;
  }

  az = azRaw / MPU_ACCEL_SCALE;

  return true;
}

bool initMPURaw()
{
  mpuWhoAmI = readMPUByte(MPU_REG_WHO_AM_I);

  Serial.print(F("MPU WHO_AM_I: 0x"));
  Serial.println(mpuWhoAmI, HEX);

  if (
    mpuWhoAmI == 0x00 ||
    mpuWhoAmI == 0xFF
  ) {
    return false;
  }

  writeMPUByte(
    MPU_REG_PWR_MGMT_1,
    0x80
  );

  delay(100);

  writeMPUByte(
    MPU_REG_PWR_MGMT_1,
    0x01
  );

  delay(100);

  writeMPUByte(
    MPU_REG_PWR_MGMT_2,
    0x00
  );

  delay(20);

  writeMPUByte(
    MPU_REG_CONFIG,
    0x03
  );

  writeMPUByte(
    MPU_REG_SMPLRT_DIV,
    0x07
  );

  writeMPUByte(
    MPU_REG_GYRO_CONFIG,
    0x00
  );

  writeMPUByte(
    MPU_REG_ACCEL_CONFIG,
    0x00
  );

  delay(300);

  float ax = 0.0;
  float ay = 0.0;
  float az = 0.0;

  float gx = 0.0;
  float gy = 0.0;
  float gz = 0.0;

  for (int i = 0; i < 10; i++) {
    if (
      readMPURawData(
        ax,
        ay,
        az,
        gx,
        gy,
        gz
      )
    ) {
      return true;
    }

    delay(100);
  }

  return false;
}

void scanI2CDevices()
{
  Serial.println();
  Serial.println(F("I2C Bus Scan:"));

  byte count = 0;

  for (
    byte address = 1;
    address < 127;
    address++
  ) {
    Wire.beginTransmission(address);

    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("  Device found at 0x"));

      if (address < 16) {
        Serial.print(F("0"));
      }

      Serial.println(address, HEX);

      count++;
    }
  }

  Serial.print(F("  Total devices detected: "));
  Serial.println(count);
  Serial.println();
}

void updateFastPrintTrigger()
{
  unsigned long now = millis();

  if (bmeOK) {
    if (!humidityReferenceReady) {
      lastHumidityForTrigger = recordHumidity;
      humidityReferenceReady = true;
    }
    else {
      float humidityDiff =
        fabs(recordHumidity - lastHumidityForTrigger);

      if (
        humidityDiff >
        HUMIDITY_CHANGE_TRIGGER
      ) {
        fastPrintUntilTime =
          now + FAST_PRINT_DURATION_MS;
      }

      lastHumidityForTrigger =
        recordHumidity;
    }
  }

  if (recordMPUReadOK) {
    if (!gyroReferenceReady) {
      lastGxForTrigger = recordGx;
      lastGyForTrigger = recordGy;
      lastGzForTrigger = recordGz;

      gyroReferenceReady = true;
    }
    else {
      float gxDiff =
        fabs(recordGx - lastGxForTrigger);

      float gyDiff =
        fabs(recordGy - lastGyForTrigger);

      float gzDiff =
        fabs(recordGz - lastGzForTrigger);

      if (
        gxDiff > GYRO_CHANGE_TRIGGER ||
        gyDiff > GYRO_CHANGE_TRIGGER ||
        gzDiff > GYRO_CHANGE_TRIGGER
      ) {
        fastPrintUntilTime =
          now + FAST_PRINT_DURATION_MS;
      }

      lastGxForTrigger = recordGx;
      lastGyForTrigger = recordGy;
      lastGzForTrigger = recordGz;
    }
  }
}

void recordDisplayData()
{

  if (adsOK) {
    recordIrLeftRaw =
      ads.readADC_SingleEnded(IR_LEFT_CH);

    recordIrCenterRaw =
      ads.readADC_SingleEnded(IR_CENTER_CH);

    recordIrRightRaw =
      ads.readADC_SingleEnded(IR_RIGHT_CH);

    recordIrLeftBlack =
      recordIrLeftRaw > IR_BLACK_THRESHOLD;

    recordIrCenterBlack =
      recordIrCenterRaw > IR_BLACK_THRESHOLD;

    recordIrRightBlack =
      recordIrRightRaw > IR_BLACK_THRESHOLD;
  }

  if (bmeOK) {
    recordTemperatureC =
      bme.readTemperature();

    recordPressureHpa =
      bme.readPressure() / 100.0;

    recordHumidity =
      bme.readHumidity();
  }

  if (mpuOK) {
    recordMPUReadOK =
      readMPURawData(
        recordAx,
        recordAy,
        recordAz,
        recordGx,
        recordGy,
        recordGz
      );

    if (recordMPUReadOK) {

      recordGx -= GYRO_OFFSET_X;
      recordGy -= GYRO_OFFSET_Y;
      recordGz -= GYRO_OFFSET_Z;

      recordGx = -recordGx;
      recordGy = -recordGy;
    }
  }
  else {
    recordMPUReadOK = false;
  }

  updateFastPrintTrigger();
}

void updateMotionState()
{
  if (!mpuOK) {
    isRunning = false;
    controlMPUReadOK = false;
    return;
  }

  float az = 0.0;

  controlMPUReadOK = readMPUAccelZ(az);

  if (!controlMPUReadOK) {
    isRunning = false;
    return;
  }

  controlZRawMps2 =
    az * STANDARD_GRAVITY;

  if (INVERT_Z_FOR_CONTROL) {
    controlZUsedMps2 =
      -controlZRawMps2;
  }
  else {
    controlZUsedMps2 =
      controlZRawMps2;
  }

  if (
    controlZUsedMps2 >
    LIFT_TRIGGER
  ) {
    isRunning = true;
  }

  if (
    controlZUsedMps2 <
    DROP_STOP
  ) {
    isRunning = false;
  }
}

void printFormalData()
{
  Serial.println(
    F("--------------------------------------------------")
  );

  /* ---------- BME280 ---------- */

  Serial.print(F("BME280: "));

  if (bmeOK) {
    Serial.print(F("Temperature="));
    Serial.print(recordTemperatureC, 2);

    Serial.print(F(" C, Humidity="));
    Serial.print(recordHumidity, 2);

    Serial.print(F(" %, Air pressure="));
    Serial.print(recordPressureHpa, 2);

    Serial.println(F(" hPa"));
  }
  else {
    Serial.println(F("NOT FOUND"));
  }

  /* ---------- MPU ---------- */

  if (recordMPUReadOK) {
    Serial.print(F("MPU Accelerometer: X="));
    Serial.print(recordAx, 3);

    Serial.print(F(" g, Y="));
    Serial.print(recordAy, 3);

    Serial.print(F(" g, Z="));
    Serial.print(recordAz, 3);

    Serial.println(F(" g"));

    Serial.print(F("MPU Gyroscope: Roll(X)="));
    Serial.print(recordGx, 3);

    Serial.print(F(" deg/s, Pitch(Y)="));
    Serial.print(recordGy, 3);

    Serial.print(F(" deg/s, Yaw(Z)="));
    Serial.print(recordGz, 3);

    Serial.println(F(" deg/s"));
  }
  else {
    Serial.println(F("MPU: READ FAILED"));
  }

  Serial.print(F("Output Mode: "));

  if (millis() < fastPrintUntilTime) {
    Serial.println(F("FAST, 0.8 s interval"));
  }
  else {
    Serial.println(F("NORMAL, 1.2 s interval"));
  }

  Serial.println(
    F("--------------------------------------------------")
  );

  Serial.println();
}

void setup()
{

  pinMode(enablePinRight, OUTPUT);
  pinMode(enablePinLeft, OUTPUT);

  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);

  pinMode(eyeLedPin, OUTPUT);
  pinMode(statusLedPin, OUTPUT);

  stopMotors();

  digitalWrite(eyeLedPin, HIGH);
  digitalWrite(statusLedPin, LOW);

  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println(F("=============================================="));
  Serial.println(F("Robot I2C Sensor Demonstration"));
  Serial.println(F("BME280 + MPU"));
  Serial.println(F("Serial Monitor: 115200 baud"));
  Serial.println(F("=============================================="));
  Serial.println();

  /* ---------- I2C ---------- */

  Wire.begin();
  Wire.setClock(100000);

  scanI2CDevices();

  /* ---------- ADS1115 ---------- */

  ads.setGain(GAIN_TWOTHIRDS);

  if (ads.begin(ADS1115_ADDR)) {
    adsOK = true;
    Serial.println(F("ADS1115: OK"));
  }
  else {
    adsOK = false;
    Serial.println(F("ADS1115: NOT FOUND"));
  }

  /* ---------- BME280 ---------- */

  if (bme.begin(BME280_ADDR1)) {
    bmeOK = true;
    Serial.println(F("BME280: OK at 0x76"));
  }
  else if (bme.begin(BME280_ADDR2)) {
    bmeOK = true;
    Serial.println(F("BME280: OK at 0x77"));
  }
  else {
    bmeOK = false;
    Serial.println(F("BME280: NOT FOUND"));
  }

  if (bmeOK) {
    bme.setSampling(
      Adafruit_BME280::MODE_NORMAL,
      Adafruit_BME280::SAMPLING_X2,
      Adafruit_BME280::SAMPLING_X16,
      Adafruit_BME280::SAMPLING_X1,
      Adafruit_BME280::FILTER_X16,
      Adafruit_BME280::STANDBY_MS_500
    );
  }

  /* ---------- MPU ---------- */

  if (initMPURaw()) {
    mpuOK = true;
    Serial.println(F("MPU: OK"));
  }
  else {
    mpuOK = false;
    Serial.println(F("MPU: NOT READY"));
  }

  Serial.println();

  recordDisplayData();
  updateMotionState();

  Serial.println(F("Setup finished."));
  Serial.println();

  isRunning = false;
  stopMotors();
}

void loop()
{
  unsigned long now = millis();

  if (
    now - lastMotionTime >=
    MOTION_INTERVAL_MS
  ) {
    lastMotionTime = now;

    updateMotionState();
  }

  if (
    now - lastRecordTime >=
    RECORD_INTERVAL_MS
  ) {
    lastRecordTime = now;

    recordDisplayData();
  }

  if (isRunning) {
    runForwardForExperiment();
    digitalWrite(statusLedPin, HIGH);
  }
  else {
    stopMotors();
    digitalWrite(statusLedPin, LOW);
  }

  unsigned long currentPrintInterval =
    NORMAL_PRINT_INTERVAL_MS;

  if (now < fastPrintUntilTime) {
    currentPrintInterval =
      FAST_PRINT_INTERVAL_MS;
  }

  if (
    now - lastPrintTime >=
    currentPrintInterval
  ) {
    lastPrintTime = now;

    printFormalData();
  }

  delay(1);
}
