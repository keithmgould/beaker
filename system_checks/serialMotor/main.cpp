#include "../../cpp_lib/constants.h"
#include "../../cpp_lib/servoMotor.h"     // common Beaker functionality

ServoMotor motorLeft  = ServoMotor(LH_ENCODER_A, LH_ENCODER_B, 1);

void setup(){
	Serial1.begin(9600); while (!Serial1) {;}
	motorLeft.init();
}

void loop(){
	motorLeft.updatePower(0.1);
}