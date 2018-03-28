## Role: Buffering the Motor Encoders

The Arduino's main purpose is to receive the motor quadrature encoders interrupt signals.
In this sense the Arduino acts as a buffer so that the Raspberry Pi is not bombarded with
the interrupt signals. When the Pi needs to know the current values of the encoders, it
asks the Arduino.


## Relevant Files:

0. balancer.ino - the main arduino file to compile (and upload to robot)
0. servoMotor.cpp - small class to help read data from encoders

## Dependencies:

0. Wire.h - allows I2C communication. (this is a core Arduino lib)
0. SoftwareSerial.h - gives us UART. (this is a core Arduino lib)
0. SabertoothSimplified.h - https://www.dimensionengineering.com/info/arduino
