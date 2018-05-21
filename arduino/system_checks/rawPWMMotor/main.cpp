#include "Arduino.h"
#include <Servo.h>
#include "../../lib/constants.h"
#include "../../lib/servoMotor.h"

/*
	This tests the most basic Servo PWM functionality.

	Note this will only work when Beaker is wired and 
	configured for PWM (vs Serial)

	90 is stopped.
	Power ranges from 0 (full forward) to 180 (full reverse)

	NOTE: This test is meant to be performed with Beaker hanging, wheels NOT touching the ground.
	
	Check: 

	If the wheels spin forward then this check passes.
	See encoder system check for clarification on forward.
  
*/


Servo servo, servo2; 

void setup()
{
  servo.attach( 11, 1000, 2000);
  servo2.attach(10, 1000, 2000);
  servo.write(80);
  servo2.write(80);
}

void loop(){}

