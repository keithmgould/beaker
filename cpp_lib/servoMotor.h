#ifndef __BEAKER_SERVOMOTOR__
#define __BEAKER_SERVOMOTOR__

#include "Arduino.h"
#include "./motor.h"
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
    raw_power = -raw_power; // its either this or swap the power wires to the motors.
    raw_power = constrain(raw_power, -1, 1);  // safety first
    raw_power *= 90; // scale to 90 in either direction (+/-)
    int power = roundf(raw_power);
    power += 90;
    Serial.print("final power to servo: ");
    Serial.println(power);
    power = constrain(power, 80, 100); // TEMPORARY!!!!-----------------------------------
    servo.write(power);
  }
};

#endif
