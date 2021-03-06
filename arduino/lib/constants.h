#ifndef __BEAKER_CONSTANTS__
#define __BEAKER_CONSTANTS__

// These constexprants have to do with Beaker's
// physical characteristics, or related computations
constexpr float WHEEL_RADIUS = .042; // in meters
constexpr int FULL_ROTATION_EDGE_EVENTS = 600; // 18.75 * 32
constexpr float THETA_OFFSET = 0.0; // due to minorly non-level IMU

constexpr float PI_OVER_ONE_EIGHTY = (PI / 180.0); // for degrees to radians
constexpr float WHEEL_CIRCUMFERECE = (TWO_PI * WHEEL_RADIUS);
constexpr float CLICKS_TO_RADIANS = (TWO_PI / FULL_ROTATION_EDGE_EVENTS);

// LED on side of robot
constexpr int INDICATOR = 8;

// pins for the motor encoders
constexpr int RH_ENCODER_A = 2; // interupt pin
constexpr int RH_ENCODER_B = 4;
constexpr int LH_ENCODER_A = 3; // interupt pin
constexpr int LH_ENCODER_B = 5;

// pin for piezo buzzer
constexpr int BUZZER_PIN = 9;

// pins for radio
constexpr int RADIO_CE = 6;
constexpr int RADIO_CSN = 7;

// pins for PWM Motor Control
constexpr int LEFT_MOTOR_DRIVER = 10;
constexpr int RIGHT_MOTOR_DRIVER = 11;

// Duration (in millisecs) for each loop.
// MAKE SURE: that the position control is
// 3-4 times slower than motor control
constexpr int MOTOR_CONTROL_TIMESTEP = 5;
constexpr int POSITION_CONTROL_TIMESTEP = 20;

// know your limits
constexpr float MAX_RADS_PER_SEC = 10;

// default Motor PID values
constexpr float MOTOR_P_PARAM = 0.025;
constexpr float MOTOR_I_PARAM = 0;
constexpr float MOTOR_D_PARAM = 0.05;

// When Beaker is Hanging, use these PID values to 
// prevent overshoot. (Above assumes much more friction/resistance)
constexpr float MOTOR_P_PARAM_HANGING = 0.001;
constexpr float MOTOR_I_PARAM_HANGING = 0.00001;
constexpr float MOTOR_D_PARAM_HANGING = 0.01;


// Used by motors for direction of rotation
constexpr int FORWARD = 1;
constexpr int BACKWARD = -1;
constexpr int LEFT = 1;
constexpr int RIGHT = -1;

#endif
