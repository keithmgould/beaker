#ifndef __BEAKER_INCLUDES__
#define __BEAKER_INCLUDES__

#include "./constants.h"    // yeah. Constants.
#include "./outputs.h"      // bells and whistles (leds, buzzer)
#include "./imu.h"			// wraps the Bno055 IMU (accelerometer, gyrometer, etc..)
#include "./waiter.h"       // waiter helper to help with...waiting
#include "./pitalk.h"		// communication with the Raspberry Pi
#include "./pid.h"        	// PID library
#include "./wheels.h"       // control get raw encoder state

#endif