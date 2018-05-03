#include "Arduino.h"
#include <Servo.h>
#include "../../cpp_lib/constants.h"
#include "../../cpp_lib/servoMotor.h"

/*
	This tests the most basic Servo PWM functionality.

	Note this will only work when Beaker is wired and 
	configured for PWM (vs Serial)

	If the wheels spin, then this check passes.
*/


Servo servo, servo2; 

void setup()
{
  servo.attach( 11, 1000, 2000);
  servo2.attach(10, 1000, 2000);
  servo.write(99);
  servo2.write(99);
}

void loop(){}

