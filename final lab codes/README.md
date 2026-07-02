# Arduino Robot Car Projects

This repository contains four Arduino programs developed for a multifunctional robot car.  
Each program is stored in a separate folder, and the `.ino` filename should match its folder name.

## Project Structure

```text
Robot_Car_Arduino/
├── Obstacle_Avoidance/
│   └── Obstacle_Avoidance.ino
├── Light_Following_PD/
│   └── Light_Following_PD.ino
├── Line_Following_PD/
│   └── Line_Following_PD.ino
├── I2C_Sensor_Demonstration/
│   └── I2C_Sensor_Demonstration.ino
└── README.md
```

## Programs

### 1. Obstacle Avoidance

File:

```text
Obstacle_Avoidance/Obstacle_Avoidance.ino
```

Main features:

- Ultrasonic distance measurement
- Servo-based left and right scanning
- Dynamic forward speed adjustment
- Smooth acceleration and deceleration
- Emergency avoidance for very close obstacles
- State-machine-based movement control

Required library:

- `Servo`  
  This library is included with the Arduino IDE.

Main pin connections:

| Module | Signal | Arduino Pin |
|---|---|---:|
| L298N | ENA | D5 |
| L298N | ENB | D6 |
| L298N | IN1 | D4 |
| L298N | IN2 | D7 |
| L298N | IN3 | D9 |
| L298N | IN4 | D10 |
| Servo | Signal | D8 |
| Ultrasonic Sensor | Trig | D11 |
| Ultrasonic Sensor | Echo | D12 |

---

### 2. Light Following with PD Control

File:

```text
Light_Following_PD/Light_Following_PD.ino
```

Main features:

- Two analog light sensors
- Exponential smoothing
- PD steering control
- Dead-band control for stable forward movement
- Minimum and maximum PWM limits
- Serial debugging output

Required libraries:

- No external library is required

Main pin connections:

| Module | Signal | Arduino Pin |
|---|---|---:|
| L298N | ENA | D9 |
| L298N | ENB | D10 |
| L298N | IN1 | D2 |
| L298N | IN2 | D3 |
| L298N | IN3 | D4 |
| L298N | IN4 | D5 |
| Left Light Sensor | Analog Output | A0 |
| Right Light Sensor | Analog Output | A1 |

Serial Monitor baud rate:

```text
9600
```

---

### 3. Line Following with PD Control

File:

```text
Line_Following_PD/Line_Following_PD.ino
```

Main features:

- Three analog infrared sensors
- ADS1115 analog-to-digital converter
- Weighted-centroid line position calculation
- PD steering control
- Line-loss detection
- Direction-locked line searching
- Low-speed recovery after the line is found
- Motor reversal only during line searching

Required libraries:

- `Wire`
- `ADS1115_WE`

Main pin connections:

| Module | Signal | Arduino Pin |
|---|---|---:|
| L298N | ENA | D5 |
| L298N | ENB | D6 |
| L298N | IN1 | D4 |
| L298N | IN2 | D7 |
| L298N | IN3 | D9 |
| L298N | IN4 | D10 |
| ADS1115 | SDA | A4 |
| ADS1115 | SCL | A5 |

ADS1115 channel assignment:

| Sensor Position | ADS1115 Channel |
|---|---|
| Left IR Sensor | A2 |
| Center IR Sensor | A1 |
| Right IR Sensor | A0 |

Default ADS1115 I2C address:

```text
0x48
```

---

### 4. I2C Sensor Demonstration

File:

```text
I2C_Sensor_Demonstration/I2C_Sensor_Demonstration.ino
```

Main features:

- ADS1115 infrared sensor acquisition
- BME280 temperature, humidity, and pressure measurement
- Raw MPU accelerometer and gyroscope register access
- Z-axis dual-threshold motor control
- Dynamic serial output frequency
- I2C device scanning

Required libraries:

- `Wire`
- `Adafruit ADS1X15`
- `Adafruit BME280`
- `Adafruit Unified Sensor`

Main pin connections:

| Module | Signal | Arduino Pin |
|---|---|---:|
| L298N | ENA | D5 |
| L298N | ENB | D6 |
| L298N | IN1 | D4 |
| L298N | IN2 | D7 |
| L298N | IN3 | D9 |
| L298N | IN4 | D10 |
| Eye LED | Signal | D3 |
| Status LED | Signal | D11 |
| I2C Devices | SDA | A4 |
| I2C Devices | SCL | A5 |

Default I2C addresses:

| Device | Address |
|---|---|
| ADS1115 | `0x48` |
| MPU | `0x68` |
| BME280 | `0x76` or `0x77` |

Serial Monitor baud rate:

```text
115200
```

## Hardware

The programs are intended for Arduino Uno or Arduino Nano compatible boards.

Typical hardware includes:

- Arduino Uno or Nano
- L298N motor driver
- Two DC motors
- Ultrasonic distance sensor
- Servo motor
- Two analog light sensors
- Three infrared line sensors
- ADS1115 ADC module
- BME280 environmental sensor
- MPU6050 or compatible MPU sensor
- LEDs
- External motor power supply

## Library Installation

Open the Arduino IDE and select:

```text
Sketch > Include Library > Manage Libraries
```

Install the required libraries for the selected program.

Recommended library names:

```text
ADS1115_WE
Adafruit ADS1X15
Adafruit BME280 Library
Adafruit Unified Sensor
```

## Usage

1. Download or clone this repository.
2. Open the folder of the required program.
3. Open the `.ino` file with the same name as the folder.
4. Install the required libraries.
5. Select the correct board and serial port in the Arduino IDE.
6. Verify the code.
7. Upload it to the Arduino board.
8. Adjust sensor thresholds, motor directions, and control parameters according to the actual hardware.

## Important Notes

- Disconnect or safely secure the motors before the first test.
- Confirm that the Arduino, sensors, motor driver, and power supply share a common ground.
- Remove the L298N ENA and ENB jumpers when PWM speed control is used.
- Do not power DC motors directly from the Arduino 5 V pin.
- Sensor thresholds may require calibration for different environments.
- Motor direction may need to be reversed depending on the wiring.
- Avoid committing passwords, API keys, access tokens, or private configuration files to the repository.

## License

This project is intended for educational and experimental use.
