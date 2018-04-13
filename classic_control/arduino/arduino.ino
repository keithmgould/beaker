// CLASSIC CONTROL -- PID

#include <StandardCplusplus.h>
#include <string>
#include <cmath>
#include <Adafruit_Sensor.h> // IMU
#include <Adafruit_BNO055.h> // IMU
#include <Wire.h> // I2C
#include "../../cpp_lib/constants.cpp"
#include "../../cpp_lib/servoMotor.cpp"

ServoMotor motorLeft(LH_ENCODER_A,LH_ENCODER_B, 1);
ServoMotor motorRight(RH_ENCODER_A,RH_ENCODER_B, -1);

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55, 40);

int kP = 0;       // holds the P parameter
int kI = 0;       // holds the I parameter
int kD = 0;       // holds the D parameter

int motorCommand = 0; // holds the final command to motor

float currentError = 0;       // error from offset. comes from phi for now
float previousError = 0;      // holds the previous timestep's error
float deltaError = 0;         // holds the derivative of the error
float accumulatedError = 0; // holds accumulated error (over time)

// following variables used to control the loop time
int loopTimeMs = 20; // desired loop time in milliseconds
long nowish = 0;
long timeDelta = 0;
long timeMarker = 0;

float xPos = 0; // meters
float lastXPos = 0;
float xVel = 0; // meters / sec
float phi = 0; // radians
float phiDot = 0; // radians / sec
float lastPhi = 0;
float theta = 0; // radians
float thetaDot = 0; // radians / sec
float lastTheta = 0;
float previousGain = 0;

// vars to hold incoming serial messages from Pi
char tmp_char;
std::string str;

void turnIndicatorLightOff(){
  digitalWrite(INDICATOR, LOW);
}

void turnIndicatorLightOn(){
  digitalWrite(INDICATOR, HIGH);
}

void updatePower(float newGain){

  // TMP Switch polarity for a TMP test!!
  newGain = -newGain;

  // safety first. ensure gain between these values
  newGain = constrain(newGain, MINIMUM_GAIN, MAXIMUM_GAIN);
  Serial.print("updatePower: ");
  Serial.println(newGain);
  previousGain = newGain;
  motorLeft.updatePower(newGain);
  motorRight.updatePower(newGain);
}

float degToRadians(float deg) {
  return deg * PI_OVER_ONE_EIGHTY;
}

// in radians. Average of both wheels
float rawPhi() {
  return (motorLeft.getPhi() + motorRight.getPhi()) / 2.0;
}

// in meters. Average of both wheels
float rawX() {
  return (motorLeft.getDistance() + motorRight.getDistance()) / 2.0;
}

// in radians
imu::Quaternion quat;
imu::Vector<3> axis;
float rawTheta() {
  quat = bno.getQuat();
  axis = quat.toEuler();
  return axis.z();
}

// calculates and stores
// theta and thetaDot
//
// argument dt is in seconds
void fetchTheta(float dt){
  theta = rawTheta();
  thetaDot = (theta - lastTheta); // no division by dt because only called once per loop
  lastTheta = theta;
}

// calculates and stores
// x and xDot (Meters and Meters/Sec)
void fetchX(){
  xPos = rawX();
  xVel = (xPos - lastXPos);
  lastXPos = xPos;
}

// calculates and stores
// phi and phiDot
void fetchPhi(){
  phi = rawPhi();
  phiDot = (phi - lastPhi);
  lastPhi = phi;
}

void fetchState(){
  fetchPhi();
  fetchX();
}

void calculateCurrentError(){
  previousError = currentError;
  currentError = theta;
}

void turnBuzzerOn(){
  digitalWrite(BUZZER_PIN, HIGH);
}

void turnBuzzerOff(){
  digitalWrite(BUZZER_PIN, LOW);
}

// Sometimes the PI leaves the motors spinning when it dies.
// When learning to balance:
//    a meter is more than enough.
void sanityCheck() {
  if(previousGain == 0){ return; }
  if(std::abs(xPos) > 0.5){
    Serial.println("\n\nFailed Sanity Check!!\n\n");
    updatePower(0);
    turnBuzzerOn();
    delay(200);
    turnBuzzerOff();
  }
}

void leftEncoderEvent() {
  sanityCheck();
  motorLeft.encoderEvent();
}

void rightEncoderEvent() {
  sanityCheck();
  motorRight.encoderEvent();
}

void beep(int durationOn, int durationOff){
  turnBuzzerOn();
  delay(durationOn);
  turnBuzzerOff();
  delay(durationOff);
}

void playInitSong(){
  beep(150, 150);
  beep(150, 75);
  beep(75, 75);
  beep(150, 150);
  beep(250, 250);
  beep(150, 150);
  beep(250, 150);
}

// kill motors and blink status indicator
// FOREVER
void errorMode(const char* input) {
  Serial.println("\n!!!!!!!!!!!!!!!!!!!!!! In Error Mode!");
  updatePower(0);
  Serial.print("Error: ");
  Serial.println(input);
  while(true){
    turnIndicatorLightOn();
    turnBuzzerOn();
    delay(300);
    turnIndicatorLightOff();
    turnBuzzerOff();
    delay(300);
  }
}

// dividing by 10 because we don't need values higher than 0-9.
void updatePIDValues(){
  kP = 0;
  kI = 0;
  kD = 0;
}

void calculateAccumulatedError(){
  // once we cross the 0 error mark, the past accumulated error
  // is no longer helping us. Reset to 0.
  if(currentError == 0){
    accumulatedError = 0;
    return;
  }

  accumulatedError += currentError;

  // helps with windup (overaccumulating the error)
  accumulatedError = constrain(accumulatedError, -20, 20);
}

// deltaError is used for the D component of the controller.
// for that we need the derivative of the error.
// since this is only called once per loop, we do not need to divide by time
void calculateDeltaError(){
  deltaError =  currentError - previousError;
}

void generateMotorCommand(){
  // this is the crux of the PID controller: adding togethe the P,I,D components
  int finalCommand = kP * currentError + kI * accumulatedError + kD * deltaError;

  // ensure we stay within bounds of motor controller.
  motorCommand = constrain(finalCommand, -255, 255);
}

void updateMotor(){

}

void sendToPi(char code, String message){
  String final;
  final = String(code) + message + "!";
  Serial.print("sending to Pi: ");
  Serial.println(final.c_str());
  Serial2.write(final.c_str());
}

void sendPIDToPi(){

}

// this is used if Arduino received an unrecognized command. The assumption is
// that the PI sent something valid, but there was a transmission error.
void resendCommand(){
  String message = " ";
  sendToPi('X', message);
}

void handle_command_from_pi(std::string message){
  Serial.print("got the message from Pi: ");
  Serial.println(message.c_str());
  char command;
  command = message[0];
  switch (command){
    case 'S': playInitSong();
              break;
    case 'U': updatePIDValues();
              break;
    default: resendCommand(); // must have been a transmission error?
  }
}

// fetch data from Pi.
// If full command is here, process it.
void checkForPiCommand(){
  while(Serial2.available()) {
    tmp_char = Serial2.read();
    if(tmp_char == '!'){
      handle_command_from_pi(str);
      str = "";
    }else{
      str += tmp_char;
    }
  }
}

// ensure main loop is TIMESTEP milliseconds long
void wait_TIMESTEP_ms(){
  while(true){
    nowish = millis();
    timeDelta = nowish - timeMarker;
    if(timeDelta < TIMESTEP){return;}
    timeMarker = nowish;
  }
}

void setup(){
  // indicator pin is an output for LED
  pinMode(INDICATOR, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // turn off indicator light while we setup
  turnIndicatorLightOff();
  turnBuzzerOff();

    // Open console serial communications
  Serial.begin(115200);
  while (!Serial) {;}
  Serial.println("\n\nBeginning initializations...");

  // Open serial comm to talk to RaspPi.
  Serial.print("Initializing comm to Rasp Pi...");
  Serial2.begin(115200);
  while (!Serial2) {;}
  Serial.println("Done!");

    // initialize motor encoders and interrupts
  Serial.print("Initializing motor encoders...");
  motorLeft.init();
  motorRight.init();
  delay(100);
  Serial.println("Done!");

  // TODO: add these to motor inits??
  Serial.print("Initializing motor interrupts...");
  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);
  Serial.println("Done!");

  // Sabertooth motor driver commanded over Serial
  Serial.print("Initializing motor driver serial connection...");
  Serial1.begin(9600);
  while (!Serial1) {;}
  Serial.println("Done!");

  //Check to see if the Inertial Sensor is wired correctly and functioning normally
  Serial.print("Initializing IMU...");
  if (!bno.begin(bno.OPERATION_MODE_IMUPLUS)) {
    Serial.println("Inertial Sensor failed, or not present");
    errorMode("could not init IMU");
  }
  bno.setExtCrystalUse(true);
  Serial.println("Done!");

  // Turn on indicator light because we are ready to rock.
  Serial.println("Done Initializing like a boss!");
  // updatePower(0); // ensure motors are off
  turnIndicatorLightOn();

  // play init song :D
  playInitSong();

  // print out two lines in case there
  // was garbage (noise) in serial setup
  Serial.println("\n");
  delay(100);
}

void loop(){
  wait_TIMESTEP_ms();
  fetchState();
  calculateCurrentError();
  calculateAccumulatedError();
  calculateDeltaError();
  generateMotorCommand();
  updateMotor();
  checkForPiCommand();
 }

