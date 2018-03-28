// pi / 180, for degrees to radians
#define PI_OVER_ONE_EIGHTY 0.017453292519943

// LED on side of robot
#define INDICATOR 8

// pins for the motor encoders
#define RH_ENCODER_A 2 // interupt pin
#define RH_ENCODER_B 4
#define LH_ENCODER_A 3 // interupt pin
#define LH_ENCODER_B 5

// pins for radio
#define RADIO_CE 6
#define RADIO_CSN 7

// Duration (in millisecs) for each loop.
// 10 => 100hz
#define TIMESTEP 10

// Minimum/Maximum values to send to motor drivers
// Should ALWAYS be between -1 and 1.
// To constrain to 50% power use -.5 and .5
#define MINIMUM_GAIN -1
#define MAXIMUM_GAIN 1

// for I2C protocol
#define SLAVE_ADDRESS 0x04
#define I2C_SEND_SIZE 17

// IMU constants
// sensor does not show 0 on balance but it should.
#define BALANCED_OFFSET 0.89
