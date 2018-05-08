#ifndef __BEAKER_SERVOMOTOR__
#define __BEAKER_SERVOMOTOR__

#include "Arduino.h"
#include "./motor.h"
#include <Servo.h>
#include <Math.h>

/*
    This class is used if the Sabertooth motor driver is configured for PWM
    communication.
*/
class ServoMotor: public Motor{
  private:

  Servo servo;

  public:

  ServoMotor(int encoderA, int encoderB, int direction): Motor(encoderA, encoderB, direction){}

  void attach(int pinNumber){
    servo.attach( pinNumber, 1000, 2000);
  }

  void updatePower(float raw_power) {
    raw_power = -raw_power; // because forward is values ranging from 0-90
    raw_power = constrain(raw_power, -1, 1);  // safety first
    raw_power *= 90; // scale to 90 in either direction (+/-)
    int power = roundf(raw_power);
    power += 90;
    servo.write(power);
  }
};

#endif
