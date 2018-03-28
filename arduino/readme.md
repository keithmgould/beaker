## Role: The Arduino acts as Beaker's nervous system:

0. receives (and buffers) the motor quadrature encoders interrupt signals.
0. commands the motors
0. communicates with the IMU
0. prepares the "state" for the Raspberry Pi (brain) and sends it up upon  request

## Why Not Skip the Arduino?

It's probably possible to skip the Arduino entirely. The reason I didn't is because having
dedicated hardware (and processing power) to lower level functions seemed (and seems) like
a good idea. The quad-encoders for example, send interrupts 300 times per single
revolution of a wheel. Two wheels mean 600 interrupts per second. I'd rather keep that
noise away from the hardware handling the neural network.

## Relevant Files:

0. arduino.ino - the main arduino file to compile (and upload to robot)
0. servoMotor.cpp - small class to encapsulate controlling motors and quad encoders

## Dependencies:

0. Wire.h - allows I2C communication. (this is a core Arduino lib)
0. SabertoothSimplified.h - https://www.dimensionengineering.com/info/arduino
