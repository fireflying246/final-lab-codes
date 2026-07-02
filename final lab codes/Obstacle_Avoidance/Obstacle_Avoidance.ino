#include <Servo.h>

/*
 * Obstacle-avoiding robot using an ultrasonic sensor, servo scanning,
 * dynamic forward speed, and state-machine control.
 */

const int enablePin1 = 5;

const int enablePin2 = 6;

const int in1Pin = 4;

const int in2Pin = 7;

const int in3Pin = 9;

const int in4Pin = 10;

const int maxSpeed = 255;

const int baseSpeed = 155;

const int backSpeed = 180;

const int pivotSpeed = 150;

const int leftSpeedOffset = 6;

const int minForwardSpeed = 140;

const int maxForwardSpeed = 165;

const int slowDistance = 30;

const int fastDistance = 50;

int currentForwardSpeed = baseSpeed;

int targetForwardSpeed = baseSpeed;

const int accelerationStep = 10;

const int decelerationStep = 30;

const int servoPin = 8;

const int inputPin = 12;

const int outputPin = 11;

Servo myservo;

const int centerAngle = 98;

const int movingScanOffset = 50;

const int movingLeftAngle = centerAngle + movingScanOffset;

const int movingRightAngle = centerAngle - movingScanOffset;

const int movingScanCount = 4;

int movingScanIndex = 0;

const int avoidLeftOffset = 95;

const int avoidRightOffset = 80;

const int wideLeftAngle = centerAngle + avoidLeftOffset;

const int wideRightAngle = centerAngle - avoidRightOffset;

const int servoMinAngle = 8;

const int servoMaxAngle = 175;

int currentServoAngle = -1;

const int servoStepAngle = 2;

const int servoStepDelay = 6;

const int frontDangerDistance = 42;

const int veryCloseDistance = 27;

const int maxValidDistance = 400;

const int obstacleConfirmTimes = 2;

int obstacleCount = 0;

const unsigned long frontCheckInterval = 80;

const unsigned long stopBeforeScanTime = 120;

const unsigned long wideServoSettleTime = 120;

const unsigned long backSmallTime = 220;

const unsigned long backLongTime = 450;

const unsigned long turnTime = 260;

const unsigned long recoverTime = 50;

int Fspeedd = maxValidDistance;

int Lspeedd = maxValidDistance;

int Rspeedd = maxValidDistance;

bool veryCloseMode = false;

enum CarState {

  STATE_FORWARD,

  STATE_STOP_BEFORE_SCAN,

  STATE_SCAN_LEFT,

  STATE_SCAN_RIGHT,

  STATE_BACKWARD,

  STATE_TURN_LEFT,

  STATE_TURN_RIGHT,

  STATE_RECOVER

};

CarState carState = STATE_FORWARD;

CarState plannedTurnState = STATE_TURN_LEFT;

unsigned long stateStartTime = 0;

unsigned long lastFrontCheckTime = 0;

//********************************************************************

// SETUP

//********************************************************************

void setup()

{

  pinMode(enablePin1, OUTPUT);

  pinMode(enablePin2, OUTPUT);

  pinMode(in1Pin, OUTPUT);

  pinMode(in2Pin, OUTPUT);

  pinMode(in3Pin, OUTPUT);

  pinMode(in4Pin, OUTPUT);

  pinMode(inputPin, INPUT);

  pinMode(outputPin, OUTPUT);

  myservo.attach(servoPin);

  safeServoWriteAngle(centerAngle);

  delay(300);

  stopMotors();

  enterState(STATE_FORWARD);

}

//********************************************************************

// LOOP

//********************************************************************

void loop()

{

  unsigned long now = millis();

  switch (carState) {

    case STATE_FORWARD:

      handleForwardState(now);

      break;

    case STATE_STOP_BEFORE_SCAN:

      handleStopBeforeScanState(now);

      break;

    case STATE_SCAN_LEFT:

      handleScanLeftState(now);

      break;

    case STATE_SCAN_RIGHT:

      handleScanRightState(now);

      break;

    case STATE_BACKWARD:

      handleBackwardState(now);

      break;

    case STATE_TURN_LEFT:

      handleTurnLeftState(now);

      break;

    case STATE_TURN_RIGHT:

      handleTurnRightState(now);

      break;

    case STATE_RECOVER:

      handleRecoverState(now);

      break;

  }

}

//********************************************************************

//********************************************************************

void safeServoWriteAngle(int targetAngle)

{

  targetAngle = constrain(targetAngle, servoMinAngle, servoMaxAngle);

  if (targetAngle == currentServoAngle) {

    return;

  }

  if (currentServoAngle < 0) {

    myservo.write(targetAngle);

    currentServoAngle = targetAngle;

    return;

  }

  if (targetAngle > currentServoAngle) {

    for (int angle = currentServoAngle; angle <= targetAngle; angle += servoStepAngle) {

      myservo.write(angle);

      delay(servoStepDelay);

    }

  }

  else {

    for (int angle = currentServoAngle; angle >= targetAngle; angle -= servoStepAngle) {

      myservo.write(angle);

      delay(servoStepDelay);

    }

  }

  currentServoAngle = targetAngle;

}

//********************************************************************

//********************************************************************

int getMovingScanAngle()

{

  if (movingScanIndex == 0) {

    return centerAngle;

  }

  else if (movingScanIndex == 1) {

    return movingLeftAngle;

  }

  else if (movingScanIndex == 2) {

    return centerAngle;

  }

  else {

    return movingRightAngle;

  }

}

bool isCenterScanAngle(int angle)

{

  return abs(angle - centerAngle) <= 5;

}

void saveMovingScanDistance(int angle, int distance)

{

  if (isCenterScanAngle(angle)) {

    Fspeedd = distance;

  }

  else if (angle > centerAngle) {

    Lspeedd = distance;

  }

  else {

    Rspeedd = distance;

  }

}

void updateMovingScanIndex()

{

  movingScanIndex++;

  if (movingScanIndex >= movingScanCount) {

    movingScanIndex = 0;

  }

}

//********************************************************************

//********************************************************************

int calculateDynamicSpeed(int distance)

{

  if (distance <= slowDistance) {

    return minForwardSpeed;

  }

  if (distance >= fastDistance) {

    return maxForwardSpeed;

  }

  int dynamicSpeed = map(distance, slowDistance, fastDistance, minForwardSpeed, maxForwardSpeed);

  dynamicSpeed = constrain(dynamicSpeed, minForwardSpeed, maxForwardSpeed);

  return dynamicSpeed;

}

//********************************************************************

//********************************************************************

void updateForwardSpeed()

{

  if (currentForwardSpeed < targetForwardSpeed) {

    currentForwardSpeed += accelerationStep;

    if (currentForwardSpeed > targetForwardSpeed) {

      currentForwardSpeed = targetForwardSpeed;

    }

  }

  else if (currentForwardSpeed > targetForwardSpeed) {

    currentForwardSpeed -= decelerationStep;

    if (currentForwardSpeed < targetForwardSpeed) {

      currentForwardSpeed = targetForwardSpeed;

    }

  }

  currentForwardSpeed = constrain(currentForwardSpeed, minForwardSpeed, maxForwardSpeed);

}

//********************************************************************

//********************************************************************

void enterState(CarState newState)

{

  carState = newState;

  stateStartTime = millis();

  switch (newState) {

    case STATE_FORWARD:

      safeServoWriteAngle(centerAngle);

      obstacleCount = 0;

      veryCloseMode = false;

      movingScanIndex = 0;

      targetForwardSpeed = baseSpeed;

      advance();

      break;

    case STATE_STOP_BEFORE_SCAN:

      stopMotors();

      safeServoWriteAngle(centerAngle);

      break;

    case STATE_SCAN_LEFT:

      stopMotors();

      safeServoWriteAngle(wideLeftAngle);

      break;

    case STATE_SCAN_RIGHT:

      stopMotors();

      safeServoWriteAngle(wideRightAngle);

      break;

    case STATE_BACKWARD:

      safeServoWriteAngle(centerAngle);

      back();

      break;

    case STATE_TURN_LEFT:

      safeServoWriteAngle(centerAngle);

      turnL();

      break;

    case STATE_TURN_RIGHT:

      safeServoWriteAngle(centerAngle);

      turnR();

      break;

    case STATE_RECOVER:

      safeServoWriteAngle(centerAngle);

      targetForwardSpeed = maxForwardSpeed;

      advance();

      break;

  }

}

//********************************************************************

//********************************************************************

void handleForwardState(unsigned long now)

{

  advance();

  if (now - lastFrontCheckTime >= frontCheckInterval) {

    lastFrontCheckTime = now;

    int scanAngle = getMovingScanAngle();

    safeServoWriteAngle(scanAngle);

    int distance = readDistanceCM();

    saveMovingScanDistance(scanAngle, distance);

    updateMovingScanIndex();

    if (isVeryCloseObstacle(distance)) {

      veryCloseMode = true;

      obstacleCount = 0;

      enterState(STATE_STOP_BEFORE_SCAN);

      return;

    }

    if (isCenterScanAngle(scanAngle)) {

      targetForwardSpeed = calculateDynamicSpeed(Fspeedd);

      if (isObstacle(Fspeedd)) {

        obstacleCount++;

      }

      else {

        obstacleCount = 0;

      }

      if (obstacleCount >= obstacleConfirmTimes) {

        veryCloseMode = false;

        enterState(STATE_STOP_BEFORE_SCAN);

      }

    }

  }

}

void handleStopBeforeScanState(unsigned long now)

{

  if (now - stateStartTime >= stopBeforeScanTime) {

    enterState(STATE_SCAN_LEFT);

  }

}

void handleScanLeftState(unsigned long now)

{

  if (now - stateStartTime >= wideServoSettleTime) {

    Lspeedd = readDistanceCM();

    enterState(STATE_SCAN_RIGHT);

  }

}

void handleScanRightState(unsigned long now)

{

  if (now - stateStartTime >= wideServoSettleTime) {

    Rspeedd = readDistanceCM();

    decideTurnDirection();

    enterState(STATE_BACKWARD);

  }

}

void handleBackwardState(unsigned long now)

{

  unsigned long backTime;

  if (veryCloseMode) {

    backTime = backLongTime;

  }

  else {

    backTime = backSmallTime;

  }

  if (now - stateStartTime >= backTime) {

    enterState(plannedTurnState);

  }

}

void handleTurnLeftState(unsigned long now)

{

  if (now - stateStartTime >= turnTime) {

    enterState(STATE_RECOVER);

  }

}

void handleTurnRightState(unsigned long now)

{

  if (now - stateStartTime >= turnTime) {

    enterState(STATE_RECOVER);

  }

}

void handleRecoverState(unsigned long now)

{

  advance();

  if (now - stateStartTime >= recoverTime) {

    Fspeedd = maxValidDistance;

    Lspeedd = maxValidDistance;

    Rspeedd = maxValidDistance;

    obstacleCount = 0;

    veryCloseMode = false;

    targetForwardSpeed = baseSpeed;

    enterState(STATE_FORWARD);

  }

}

//********************************************************************

//********************************************************************

void decideTurnDirection()

{

  if (Lspeedd < frontDangerDistance && Rspeedd < frontDangerDistance) {

    if (Lspeedd >= Rspeedd) {

      plannedTurnState = STATE_TURN_LEFT;

    }

    else {

      plannedTurnState = STATE_TURN_RIGHT;

    }

  }

  else if (Lspeedd > Rspeedd) {

    plannedTurnState = STATE_TURN_LEFT;

  }

  else {

    plannedTurnState = STATE_TURN_RIGHT;

  }

}

//********************************************************************

//********************************************************************

int readDistanceCM()

{

  digitalWrite(outputPin, LOW);

  delayMicroseconds(2);

  digitalWrite(outputPin, HIGH);

  delayMicroseconds(10);

  digitalWrite(outputPin, LOW);

  unsigned long duration = pulseIn(inputPin, HIGH, 30000);

  if (duration == 0) {

    return maxValidDistance;

  }

  int distance = duration / 58;

  if (distance <= 0 || distance > maxValidDistance) {

    distance = maxValidDistance;

  }

  return distance;

}

bool isObstacle(int distance)

{

  return distance > 2 && distance <= frontDangerDistance;

}

bool isVeryCloseObstacle(int distance)

{

  return distance > 2 && distance <= veryCloseDistance;

}

//********************************************************************

//********************************************************************

bool isPwmPin(int pin)

{

  return pin == 3 || pin == 5 || pin == 6 || pin == 9 || pin == 10 || pin == 11;

}

void writeEnable(int pin, int speedValue)

{

  speedValue = constrain(speedValue, 0, 255);

  if (isPwmPin(pin)) {

    analogWrite(pin, speedValue);

  }

  else {

    if (speedValue > 0) {

      digitalWrite(pin, HIGH);

    }

    else {

      digitalWrite(pin, LOW);

    }

  }

}

//********************************************************************

//********************************************************************

void rightMotorForward(int speedValue)

{

  speedValue = constrain(speedValue, 0, 255);

  writeEnable(enablePin1, speedValue);

  digitalWrite(in1Pin, HIGH);

  digitalWrite(in2Pin, LOW);

}

void rightMotorBackward(int speedValue)

{

  speedValue = constrain(speedValue, 0, 255);

  writeEnable(enablePin1, speedValue);

  digitalWrite(in1Pin, LOW);

  digitalWrite(in2Pin, HIGH);

}

void rightMotorStop()

{

  writeEnable(enablePin1, 0);

  digitalWrite(in1Pin, LOW);

  digitalWrite(in2Pin, LOW);

}

void leftMotorForward(int speedValue)

{

  speedValue = constrain(speedValue - leftSpeedOffset, 0, 255);

  writeEnable(enablePin2, speedValue);

  digitalWrite(in3Pin, HIGH);

  digitalWrite(in4Pin, LOW);

}

void leftMotorBackward(int speedValue)

{

  speedValue = constrain(speedValue - leftSpeedOffset, 0, 255);

  writeEnable(enablePin2, speedValue);

  digitalWrite(in3Pin, LOW);

  digitalWrite(in4Pin, HIGH);

}

void leftMotorStop()

{

  writeEnable(enablePin2, 0);

  digitalWrite(in3Pin, LOW);

  digitalWrite(in4Pin, LOW);

}

void stopMotors()

{

  rightMotorStop();

  leftMotorStop();

}

//********************************************************************

//********************************************************************

void advance()

{

  updateForwardSpeed();

  rightMotorForward(currentForwardSpeed);

  leftMotorForward(currentForwardSpeed);

}

void back()

{

  rightMotorBackward(backSpeed);

  leftMotorBackward(backSpeed);

}

void turnL()

{

  rightMotorForward(pivotSpeed);

  leftMotorBackward(pivotSpeed);

}

void turnR()

{

  rightMotorBackward(pivotSpeed);

  leftMotorForward(pivotSpeed);

}
