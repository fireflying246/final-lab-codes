/*
 * PD light-following control for two bidirectional motors.
 */

const int enablePin1 = 9;   // Right motor PWM
const int enablePin2 = 10;  // Left motor PWM
const int in1Pin = 2;
const int in2Pin = 3;
const int in3Pin = 4;
const int in4Pin = 5;

const int LeftSensorPin = A0;
const int RightSensorPin = A1;

float SensorLeft = 0.0;
float SensorRight = 0.0;

const int baseSpeed = 100;
const int maxSpeed = 160;
const int minSpeed = 80;
const int deadBand = 60;
const int minLightSum = 150;
const float Kp = 0.4;
const float Kd = 0.3;
const float alpha = 0.25;

const int leftBias = 0;

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
int error = 0;
int lastError = 0;
int dError = 0;
int turn = 0;
int rawLeft = 0;
int rawRight = 0;
int totalLight = 0;

void setMotor(int leftSpeed, int rightSpeed, boolean reverse);

void setup()
{
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(enablePin1, OUTPUT);
  pinMode(enablePin2, OUTPUT);

  Serial.begin(9600);
}

void loop()
{
  boolean reverse = false;

  rawLeft = 1023 - analogRead(LeftSensorPin);
  delay(1);
  rawRight = 1023 - analogRead(RightSensorPin);
  delay(1);

  // Exponential smoothing reduces sensor noise.
  SensorLeft = alpha * SensorLeft + (1.0 - alpha) * rawLeft;
  SensorRight = alpha * SensorRight + (1.0 - alpha) * rawRight;

  error = (int)(SensorLeft - SensorRight);
  dError = error - lastError;
  totalLight = (int)(SensorLeft + SensorRight);

  if (totalLight < minLightSum) {
    leftMotorSpeed = 0;
    rightMotorSpeed = 0;
    lastError = 0;
  }
  else {
    if (abs(error) < deadBand) {
      leftMotorSpeed = baseSpeed;
      rightMotorSpeed = baseSpeed;
    }
    else {
      turn = (int)(Kp * error + Kd * dError);

      int maxTurn = maxSpeed - baseSpeed;
      turn = constrain(turn, -maxTurn, maxTurn);

      leftMotorSpeed = baseSpeed - turn;
      rightMotorSpeed = baseSpeed + turn;

      leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed);
      rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);

      if (leftMotorSpeed > 0 && leftMotorSpeed < minSpeed) {
        leftMotorSpeed = minSpeed;
      }
      if (rightMotorSpeed > 0 && rightMotorSpeed < minSpeed) {
        rightMotorSpeed = minSpeed;
      }
    }

    lastError = error;
  }

  setMotor(leftMotorSpeed, rightMotorSpeed, reverse);

  Serial.print("L=");
  Serial.print(SensorLeft);
  Serial.print("  R=");
  Serial.print(SensorRight);
  Serial.print("  err=");
  Serial.print(error);
  Serial.print("  dErr=");
  Serial.print(dError);
  Serial.print("  turn=");
  Serial.print(turn);
  Serial.print("  LS_cmd=");
  Serial.print(leftMotorSpeed);
  Serial.print("  LS_out=");
  Serial.print(max(0, leftMotorSpeed - leftBias));
  Serial.print("  RS=");
  Serial.println(rightMotorSpeed);

  delay(20);
}

void setMotor(int leftSpeed, int rightSpeed, boolean reverse)
{
  int actualLeftSpeed = leftSpeed - leftBias;
  if (actualLeftSpeed < 0) {
    actualLeftSpeed = 0;
  }

  analogWrite(enablePin1, rightSpeed);
  digitalWrite(in1Pin, !reverse);
  digitalWrite(in2Pin, reverse);

  analogWrite(enablePin2, actualLeftSpeed);
  digitalWrite(in3Pin, !reverse);
  digitalWrite(in4Pin, reverse);
}
