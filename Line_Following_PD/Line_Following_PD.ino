#include <Wire.h>
#include <ADS1115_WE.h>

/*
 * ADS1115-based line follower using weighted-centroid error and PD control.
 * Lost-line search follows the last reliable direction and uses a recovery mode.
 */

#define I2C_ADDRESS 0x48

ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);

const int enablePin1 = 5;   // Right motor ENA
const int enablePin2 = 6;   // Left motor ENB
const int in1Pin = 4;
const int in2Pin = 7;
const int in3Pin = 9;
const int in4Pin = 10;

boolean reverseL = false;
boolean reverseR = false;

const int baseSpeed = 100;
const int maxSpeed = 125;
const int minRunSpeed = 35;
const int leftSpeedOffset = 8;

float Kp = 0.44;
float Kd = 1.70;

const int normalMaxCorrection = 91;
const int derivativeLimit = 40;
const int centerDeadBand = 6;

const long LINE_LOST_SUM = 100;
const int LINE_LOST_CONFIRM = 3;
const int lostSearchOuterSpeed = 110;
const int lostSearchInnerReverseSpeed = -45;
const int lostNoDirectionSpeed = 65;
const unsigned long lostSearchMinTime = 120;
const int lineFoundConfirmNeed = 3;
const unsigned long recoveryTime = 220;
const int recoveryBaseSpeed = 60;
const int recoveryMaxCorrection = 5;

ADS1115_MUX irChannels[3] = {
  ADS1115_COMP_2_GND,  // Left
  ADS1115_COMP_1_GND,  // Center
  ADS1115_COMP_0_GND   // Right
};

int sensorWhiteMV[3] = {940, 940, 940};
const bool lineIsHigh = true;
const int sensorPos[3] = {-100, 0, 100};

int irRawMV[3] = {0, 0, 0};
int irFilteredMV[3] = {0, 0, 0};
long blackness[3] = {0, 0, 0};

const int filterN = 1;

const int MODE_NORMAL = 0;
const int MODE_LOST_SEARCH = 1;
const int MODE_RECOVERY = 2;

int controlMode = MODE_NORMAL;
unsigned long modeStartTime = 0;

float error = 0.0;
float lastError = 0.0;
float lastReliableError = 0.0;

bool lineLost = false;
int lineLostCount = 0;
int lockedLostSearchDirection = 0;
int lineFoundStableCount = 0;

int leftServoSpeed = 0;
int rightServoSpeed = 0;

void Scan();
void ComputeError();
void UpdateControl();
void StartLostSearchMode();
void HandleLostSearchMode();
void RunLostSearch();
int GetLostSearchDirection();
void StartRecoveryMode();
void HandleRecoveryMode();
void RunPDControl(int currentBaseSpeed, int maxCorrection, bool allowReverse);
void Drive(int leftSpeed, int rightSpeed);
int clampSignedSpeed(int value);
void setMotor(int leftSpeedValue, int rightSpeedValue, boolean leftReverse, boolean rightReverse);
bool isPwmPin(int pin);
void writeEnable(int pin, int speedValue);
float readChannel(ADS1115_MUX channel);
float absFloat(float value);

void setup()
{
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(enablePin1, OUTPUT);
  pinMode(enablePin2, OUTPUT);

  delay(500);

  Wire.begin();
  Wire.setClock(400000);

  if (!adc.init()) {
    while (true) {
      Drive(0, 0);
      delay(500);
    }
  }

  adc.setVoltageRange_mV(ADS1115_RANGE_6144);
  adc.setConvRate(ADS1115_860_SPS);
}

void loop()
{
  Scan();
  ComputeError();
  UpdateControl();
  Drive(leftServoSpeed, rightServoSpeed);
}

void Scan()
{
  static bool firstScan = true;

  for (int i = 0; i < 3; i++) {
    float voltage = readChannel(irChannels[i]);
    irRawMV[i] = (int)(voltage * 1000.0);

    if (firstScan) {
      irFilteredMV[i] = irRawMV[i];
    }
    else {
      irFilteredMV[i] =
        (irFilteredMV[i] * (filterN - 1) + irRawMV[i]) / filterN;
    }

    long value;

    if (lineIsHigh) {
      value = (long)irFilteredMV[i] - sensorWhiteMV[i];
    }
    else {
      value = (long)sensorWhiteMV[i] - irFilteredMV[i];
    }

    blackness[i] = (value > 0) ? value : 0;
  }

  firstScan = false;
}

void ComputeError()
{
  long sum = blackness[0] + blackness[1] + blackness[2];

  if (sum < LINE_LOST_SUM) {
    lineLostCount++;

    if (lineLostCount >= LINE_LOST_CONFIRM) {
      lineLost = true;

      if (lastReliableError > 10.0) {
        error = 100.0;
      }
      else if (lastReliableError < -10.0) {
        error = -100.0;
      }
      else {
        error = lastError;
      }
    }
    else {
      lineLost = false;
      error = lastError;
    }

    return;
  }

  lineLost = false;
  lineLostCount = 0;

  long weightedSum = 0;
  weightedSum += (long)sensorPos[0] * blackness[0];
  weightedSum += (long)sensorPos[1] * blackness[1];
  weightedSum += (long)sensorPos[2] * blackness[2];

  error = (float)weightedSum / (float)sum;
  error = constrain(error, -100.0, 100.0);

  if (absFloat(error) <= centerDeadBand) {
    error = 0.0;
  }

  if (error > 15.0 || error < -15.0) {
    lastReliableError = error;
  }
}

void UpdateControl()
{
  if (controlMode == MODE_NORMAL) {
    if (lineLost) {
      StartLostSearchMode();
      RunLostSearch();
      return;
    }

    RunPDControl(baseSpeed, normalMaxCorrection, false);
    return;
  }

  if (controlMode == MODE_LOST_SEARCH) {
    HandleLostSearchMode();
    return;
  }

  if (controlMode == MODE_RECOVERY) {
    HandleRecoveryMode();
  }
}

void StartLostSearchMode()
{
  controlMode = MODE_LOST_SEARCH;
  modeStartTime = millis();
  lineFoundStableCount = 0;
  lockedLostSearchDirection = GetLostSearchDirection();
  lastError = error;
}

void HandleLostSearchMode()
{
  unsigned long now = millis();

  if (now - modeStartTime >= lostSearchMinTime) {
    if (!lineLost) {
      lineFoundStableCount++;
    }
    else {
      lineFoundStableCount = 0;
    }
  }

  if (lineFoundStableCount >= lineFoundConfirmNeed) {
    StartRecoveryMode();
    RunPDControl(recoveryBaseSpeed, recoveryMaxCorrection, false);
    return;
  }

  RunLostSearch();
}

void RunLostSearch()
{
  if (lockedLostSearchDirection > 0) {
    leftServoSpeed = lostSearchOuterSpeed;
    rightServoSpeed = lostSearchInnerReverseSpeed;
  }
  else if (lockedLostSearchDirection < 0) {
    leftServoSpeed = lostSearchInnerReverseSpeed;
    rightServoSpeed = lostSearchOuterSpeed;
  }
  else {
    leftServoSpeed = lostNoDirectionSpeed;
    rightServoSpeed = lostNoDirectionSpeed;
  }
}

int GetLostSearchDirection()
{
  if (lastReliableError > 15.0) {
    return 1;
  }
  if (lastReliableError < -15.0) {
    return -1;
  }
  if (lastError > 15.0) {
    return 1;
  }
  if (lastError < -15.0) {
    return -1;
  }
  return 0;
}

void StartRecoveryMode()
{
  controlMode = MODE_RECOVERY;
  modeStartTime = millis();
  lineFoundStableCount = 0;
  lockedLostSearchDirection = 0;
  lastError = error;
}

void HandleRecoveryMode()
{
  unsigned long now = millis();

  if (lineLost) {
    StartLostSearchMode();
    RunLostSearch();
    return;
  }

  if (now - modeStartTime >= recoveryTime) {
    controlMode = MODE_NORMAL;
    lastError = error;
    RunPDControl(baseSpeed, normalMaxCorrection, false);
    return;
  }

  RunPDControl(recoveryBaseSpeed, recoveryMaxCorrection, false);
}

void RunPDControl(int currentBaseSpeed, int maxCorrection, bool allowReverse)
{
  float derivative = error - lastError;
  derivative = constrain(derivative, -derivativeLimit, derivativeLimit);

  float correction = Kp * error + Kd * derivative;
  correction = constrain(correction, -maxCorrection, maxCorrection);

  leftServoSpeed = currentBaseSpeed + (int)correction;
  rightServoSpeed = currentBaseSpeed - (int)correction;

  lastError = error;

  if (!allowReverse) {
    leftServoSpeed = constrain(leftServoSpeed, 0, maxSpeed);
    rightServoSpeed = constrain(rightServoSpeed, 0, maxSpeed);

    if (leftServoSpeed > 0 && leftServoSpeed < minRunSpeed) {
      leftServoSpeed = minRunSpeed;
    }

    if (rightServoSpeed > 0 && rightServoSpeed < minRunSpeed) {
      rightServoSpeed = minRunSpeed;
    }
  }
}

void Drive(int leftSpeed, int rightSpeed)
{
  int left = clampSignedSpeed(leftSpeed);
  int right = clampSignedSpeed(rightSpeed);

  reverseL = false;
  reverseR = false;

  if (left < 0) {
    left = -left;
    reverseL = true;
  }

  if (right < 0) {
    right = -right;
    reverseR = true;
  }

  setMotor(left, right, reverseL, reverseR);
}

int clampSignedSpeed(int value)
{
  if (value > maxSpeed) {
    value = maxSpeed;
  }
  else if (value < -maxSpeed) {
    value = -maxSpeed;
  }

  if (value > 0 && value < minRunSpeed) {
    value = minRunSpeed;
  }
  else if (value < 0 && value > -minRunSpeed) {
    value = -minRunSpeed;
  }

  return value;
}

void setMotor(
  int leftSpeedValue,
  int rightSpeedValue,
  boolean leftReverse,
  boolean rightReverse
)
{
  leftSpeedValue = constrain(leftSpeedValue, 0, maxSpeed);
  rightSpeedValue = constrain(rightSpeedValue, 0, maxSpeed);

  int correctedLeftSpeed = leftSpeedValue - leftSpeedOffset;
  correctedLeftSpeed = constrain(correctedLeftSpeed, 0, maxSpeed);

  writeEnable(enablePin1, rightSpeedValue);
  digitalWrite(in1Pin, !rightReverse);
  digitalWrite(in2Pin, rightReverse);

  writeEnable(enablePin2, correctedLeftSpeed);
  digitalWrite(in3Pin, !leftReverse);
  digitalWrite(in4Pin, leftReverse);
}

bool isPwmPin(int pin)
{
  return pin == 3 || pin == 5 || pin == 6 ||
         pin == 9 || pin == 10 || pin == 11;
}

void writeEnable(int pin, int speedValue)
{
  speedValue = constrain(speedValue, 0, 255);

  if (isPwmPin(pin)) {
    analogWrite(pin, speedValue);
  }
  else {
    digitalWrite(pin, speedValue > 0 ? HIGH : LOW);
  }
}

float readChannel(ADS1115_MUX channel)
{
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();

  while (adc.isBusy()) {
    delay(0);
  }

  return adc.getResult_V();
}

float absFloat(float value)
{
  return value >= 0.0 ? value : -value;
}
