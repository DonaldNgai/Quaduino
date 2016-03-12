# Quaduino

A quadcopter controlled by an Arduino Uno capable of automated hovering.

## Parts
**Frame** - https://www.hobbyking.com/hobbyking/store/uh_viewItem.asp?idProduct=66628

**Batteries** - https://www.hobbyking.com/hobbyking/store/uh_viewItem.asp?idProduct=8934

**Motors and Propellers** - https://www.hobbyking.com/hobbyking/store/uh_viewItem.asp?idProduct=43884

**Power Distribution Board** - https://www.hobbyking.com/hobbyking/store/uh_viewItem.asp?idProduct=23140

**Electronic Speed Controllers (ESC's)** - https://www.hobbyking.com/hobbyking/store/uh_viewItem.asp?idProduct=58264

**Charger** - https://www.hobbyking.com/hobbyking/store/uh_viewItem.asp?idProduct=49338

###Controller (IMU)
**Arduino UNO** - https://www.arduino.cc/en/Main/ArduinoBoardUno

**Accelerometer and Gyroscope (MPU6050)** - http://www.ebay.com/itm/6DOF-MPU-6050-Module-3-Axis-Gyroscope-Accelerometer-Module-for-Arduino-MPU-6050-/381374932942?

**Barometer (BMP180)** - http://www.ebay.com/itm/BMP180-Replace-BMP085-Digital-Barometric-Pressure-Sensor-Board-Module-Arduino-/261719866003?ssPageName=ADME:L:OU:CA:3160

###Communication
**Bluetooth receiver (HC06)** - http://www.ebay.com/itm/Wireless-Serial-4-Pin-Bluetooth-RF-Transceiver-Module-HC-06-RS232-With-backplane-/200924726178?ssPageName=ADME:X:RRIRTB:US:3160

**Android Phone** - I used an HTC One M8

## Installation

1.	Make sure the quadcopter is wired correctly based on the diagram shown here.
2.	Upload the Arduino code onto the Arduino.
3.	Install the application onto an Android device.

## Usage

###Changing PID values
1.	Return to the main screen on the Android application
2.	Reset the Arduino UNO by pressing the reset button onboard
3.	Wait 2 seconds
4.	Click on "Set Values"
5.	The LED should light up if this was done correctly, if not, exit back to the main screen and repeat step 4.
6.	Set the PID values and then click the "Send Values" button
IMPORTANT: Only set up to 3 PID values at once to be safe
7.	If the values were successfully set, the ESC's will now be armed and thus will make a noise.

###Operating the Quadcopter
1.	Check the "Arm" checkbox. The ESC's should arm themselves and make a noise.
2.	At this point, changing the power slider will not take effect as the "Power" checkbox is unchecked. It is recommended to make sure the power slider is at 0.
3.	Check the "Power" checkbox to gain control of the power of the motors
4.	Check the "Control" checkbox if you wish to control the quadcopter using the phone's orientation. It is recommended to click the "Calibrate" button before this to set the current orientation of the phone as the neutral position.

If either the "Arm" or "Power" checkboxes are unchecked during flight, the Quadcopter will enter into hover mode.

## Credits

###Software Used:
**Arduino** - https://www.arduino.cc/en/Main/Software

**MIT App Inventor 2** - http://ai2.appinventor.mit.edu/

**Eagle PCB Design Software** - http://www.cadsoftusa.com/download-eagle/?CMP=KNC-GUS-FUS-GEN-SUP-CAD-Eeagle

###Tutorials Used/Code Used:
**Hardware Reference** - http://www.instructables.com/id/DIY-ARDUINO-FLIGHT-CONTROLLER/
http://forum.arduino.cc/index.php?topic=184503.0

**Testing MPU6050** - http://diyhacking.com/arduino-mpu-6050-imu-sensor-tutorial/
http://playground.arduino.cc/Main/MPU-6050
https://www.sparkfun.com/products/11028

**Testing BMP180** - https://learn.sparkfun.com/tutorials/bmp180-barometric-pressure-sensor-hookup-

**IMU and PID code reference** - https://github.com/baselsw/BlueCopter

**ESC and Motor operation** - http://www.instructables.com/id/ESC-Programming-on-Arduino-Hobbyking-ESC/
http://forum.arduino.cc/index.php?topic=20184.0



