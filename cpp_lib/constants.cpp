// pi / 180, for degrees to radians
#define PI_OVER_ONE_EIGHTY 0.017453292519943

// LED on side of robot
#define INDICATOR 8

// pins for the motor encoders
#define RH_ENCODER_A 2 // interupt pin
#define RH_ENCODER_B 4
#define LH_ENCODER_A 3 // interupt pin
#define LH_ENCODER_B 5

// pin for piezo buzzer
#define BUZZER_PIN 9

// pins for radio
#define RADIO_CE 6
#define RADIO_CSN 7

// Duration (in millisecs) for each loop.
// 20 => 50hz
#define TIMESTEP 20

// Minimum/Maximum values to send to motor drivers
// Should ALWAYS be between -1 and 1.
// To constrain to 50% power use -.5 and .5
#define MINIMUM_GAIN -1
#define MAXIMUM_GAIN 1
