#ifndef __BEAKER_CONSTANTS__
#define __BEAKER_CONSTANTS__

// pi / 180, for degrees to radians
#define PI_OVER_ONE_EIGHTY 0.017453292519943

// LED on side of robot
#define INDICATOR 8

// pins for the motor encoders
#define RH_ENCODER_A 3 // interupt pin
#define RH_ENCODER_B 5
#define LH_ENCODER_A 2 // interupt pin
#define LH_ENCODER_B 4

// pin for piezo buzzer
#define BUZZER_PIN 9

// pins for radio
#define RADIO_CE 6
#define RADIO_CSN 7

// Duration (in millisecs) for each loop.
// MAKE SURE: that the position control is
// 3-4 times slower than motor control
#define MOTOR_CONTROL_TIMESTEP 5
#define POSITION_CONTROL_TIMESTEP 20

// Minimum/Maximum values to send to motor drivers
// Should ALWAYS be between -1 and 1.
// To constrain to 50% power use -.5 and .5
#define MINIMUM_GAIN -1
#define MAXIMUM_GAIN 1

// Used by Wheels object
#define THETA_OFFSET 0.01 // due to minorly non-level IMU
#define MOTOR_P_PARAM 0.015
#define MOTOR_I_PARAM 0
#define MOTOR_D_PARAM 0.05

#endif
