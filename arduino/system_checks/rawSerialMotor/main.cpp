#include "Arduino.h"

/*
	This tests the most basic Serial Motor functionality.

	Note this will only work when Beaker is wired and 
	configured for Serial (vs PWM)

  See full description in serialMotor.h, but briefly:

  We have seven bits (0-127) so:
  range between -63 and 63.
  -63 is full reverse
  0 is stop
  63 is full forward


  NOTE: This test is meant to be performed with Beaker hanging, wheels NOT touching the ground.
	
	Check: 

	If the wheels spin forward then this check passes.
	See 'encoders' system check for clarification on "forward."
  
*/


void setup()
{
		Serial1.begin(9600); while (!Serial1) { delay(1); }
		int power = 5;
    byte command = 64 - power;
    Serial1.write(command);
    command = 192 - power;
    Serial1.write(command);
}

void loop(){}

