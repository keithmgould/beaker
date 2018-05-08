#ifndef __BEAKER_SERIALMOTOR__
#define __BEAKER_SERIALMOTOR__
#include "Arduino.h"
#include "./motor.h"
#include <Math.h>

/*
    This class is used if the Sabertooth motor driver is configured for serial
    communication.
*/
class SerialMotor: public Motor{
  public:

  SerialMotor(int encoderA, int encoderB, int direction): Motor(encoderA, encoderB, direction){}

  /*
  Because Sabertooth controls two motors with one 8 byte character,
  when operating in Simplified Serial mode, each motor has 7 bits of resolution.
  Sending a character between 1 and 127 will control motor 1. 1 is full reverse,
  64 is stop and 127 is full forward. Sending a character between 128 and 255
  will control motor 2. 128 is full reverse, 192 is stop and 255 is full forward.
  Character 0 (hex 0x00) is a special case.
  Sending this character will shut down both motors.

  We have seven bits (0-127) so:
  range between -63 and 63.
  -63 is full reverse
  0 is stop
  63 is full forward

  input is a float between -1 and 1
  */
  void updatePower(float raw_power) {
    raw_power *= -1; // keep this or reverse the wiring.
    raw_power = constrain(raw_power, -1, 1);  // safety first
    raw_power *= 63;
    int power = roundf(raw_power);
    byte command = tickDirection > 0 ? 64 + power : 192 + power;
    Serial1.write(command);
  }
};

#endif
