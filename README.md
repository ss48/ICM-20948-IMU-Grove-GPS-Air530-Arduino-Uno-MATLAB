STEP-BY-STEP GUIDE to get ICM-20948 IMU + Grove GPS Air530 + Arduino Uno + MATLAB working together.

This includes:

Hardware wiring

Installing the correct Arduino libraries

Uploading a ready-to-use Arduino sketch

MATLAB code to read data, run complementary filter, and fuse GPS + IMU



STEP 1 — Wire the hardware
 ICM-20948 (I²C)

Connect the IMU to UNO using I²C:

ICM-20948 Pin	Arduino Uno Pin
SDA	A4
SCL	A5
VCC	3.3V
GND	GND
 NOTE: Most ICM-20948 boards are 3.3V only.

 Grove GPS Air530 (UART)

To avoid clashing with USB Serial, we use SoftwareSerial.

Wire like this:

Air530 Pin	Arduino Uno Pin
TX	D10 (RX for Arduino)
RX	D11 (TX – optional)
VCC	5V
GND	GND
 STEP 2 — Install required Arduino libraries

Open Arduino IDE → Sketch → Include Library → Manage Libraries…

Search and install:

 TinyGPSPlus

(For GPS parsing)

 SparkFun ICM-20948 IMU

(or whichever ICM-20948 library you have)

Looking for one of these:

“SparkFun ICM-20948 9DoF IMU”

“Seeed ICM-20948”

“DFRobot ICM-20948”


