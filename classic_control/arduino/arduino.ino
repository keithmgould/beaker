// CLASSIC CONTROL -- PID

#include <StandardCplusplus.h>
#include <sstream>      // std::istringstream
// #include <string>

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

float kP = 0;       // holds the P parameter
float kI = 0;       // holds the I parameter
float kD = 0;       // holds the D parameter

float motorCommand = 0; // holds the final command to motor

float currentError = 0;       // error from offset. comes from phi for now
float previousError = 0;      // holds the previous timestep's error
float deltaError = 0;         // holds the derivative of the error
float accumulatedError = 0; // holds accumulated error (over time)

// following variables used to control the loop times
long positionTimeMarker = 0;
long motorTimeMarker = 0;
long lastPrintNow = 0; // only used to verify final loop time

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

void printStatus(){
  String str = "";
  long printNow = millis();
  str += String(printNow - lastPrintNow);
  str += ", " + String(theta,5);
  str += ", " + String(accumulatedError, 5);
  str += ", " + String(deltaError, 5);
  str += ", " + String(motorCommand, 1);
  lastPrintNow = printNow;
  Serial.println(str);
}

void turnIndicatorLightOff(){
  digitalWrite(INDICATOR, LOW);
}

void turnIndicatorLightOn(){
  digitalWrite(INDICATOR, HIGH);
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
float rawTheta() {
  imu::Quaternion quat;
  imu::Vector<3> axis;

  quat = bno.getQuat();
  axis = quat.toEuler();
  return axis.z();
}

// calculates and stores theta and thetaDot
// rads and rads/sec
void fetchTheta(float dt, float timestep){
  theta = rawTheta() + THETA_OFFSET;
  thetaDot = (theta - lastTheta) / (dt / timestep);
  lastTheta = theta;
}

// calculates and stores x and xDot
// meters and meters/sec
void fetchX(float dt, float timestep){
  xPos = rawX();
  xVel = (xPos - lastXPos) / (dt / timestep);
  lastXPos = xPos;
}

// calculates and stores phi and phiDot
// rads and rads/sec
void fetchPhi(float dt, float timestep){
  phi = rawPhi();
  phiDot = (phi - lastPhi) / (dt / timestep);
  lastPhi = phi;
}

void fetchState(float dt, float timestep){
  fetchTheta(dt, timestep);
  fetchPhi(dt, timestep);
  fetchX(dt, timestep);
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

void beep(int durationOn, int durationOff){
  turnBuzzerOn();
  delay(durationOn);
  turnBuzzerOff();
  delay(durationOff);
}

void playInitSong(){
  beep(100, 50);
  beep(100, 50);
  beep(100, 50);
}

void updateMotor(){
  // safety first. ensure gain between these values
  float newGain = constrain(motorCommand, -1, 1);

  if(newGain == previousGain){ return; }

  previousGain = newGain;

  motorLeft.updatePower(newGain);
  motorRight.updatePower(newGain);
}

void stopMotors() {
  motorCommand = 0;
  updateMotor();
}

void leftEncoderEvent() {
  motorLeft.encoderEvent();
}

void rightEncoderEvent() {
  motorRight.encoderEvent();
}

// kill motors and blink status indicator
// FOREVER
void errorMode(const char* input) {
  Serial.println("\n!!!!!!!!!!!!!!!!!!!!!! In Error Mode!");
  stopMotors();
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

void updatePIDValues(std::string message){
  Serial.println("Updating PID Values!!!");
  Serial.println(message.c_str());

  float values[3];
  std::istringstream ss( message );
  std::copy(
    std::istream_iterator <float> ( ss ),
    std::istream_iterator <float> (),
    values
    );


  kP = values[0];
  kI = values[1];
  kD = values[2];
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
  accumulatedError = constrain(accumulatedError, -1, 1);
}

// deltaError is used for the D component of the controller.
// for that we need the derivative of the error.
// since this is only called once per loop, we do not need to divide by time
void calculateDeltaError(){
  deltaError =  currentError - previousError;
}

void generateMotorCommand(){
  // this is the crux of the PID controller: adding togethe the P,I,D components
  float finalCommand = kP * currentError + kI * accumulatedError + kD * deltaError;

  // ensure we stay within bounds of motor controller.
  motorCommand = constrain(finalCommand, -1, 1);
}

void sendToPi(char code, String message){
  String final;
  final = String(code) + message + "!";
  Serial.print("sending to Pi: ");
  Serial.println(final.c_str());
  Serial2.write(final.c_str());
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
    case 'U': updatePIDValues(message.substr(1));
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

long wait_loop_ms(long &marker, int timestep){
  long nowish = 0;
  long timeDelta = 0;

  while(true){
    nowish = millis();
    timeDelta = nowish - marker;
    if(timeDelta < timestep){ continue; }
    marker = nowish;
    break;
  }

  return timeDelta;
}

long wait_POSITION_CONTROL_TIMESTEP_ms(){
  return wait_loop_ms(positionTimeMarker, POSITION_CONTROL_TIMESTEP);
}

long wait_MOTOR_CONTROL_TIMESTEP_ms(){
  return wait_loop_ms(motorTimeMarker, MOTOR_CONTROL_TIMESTEP);
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
  long motor_delta = wait_MOTOR_CONTROL_TIMESTEP_ms();
  fetchState(motor_delta, MOTOR_CONTROL_TIMESTEP);
  generateMotorCommand();
  wait_POSITION_CONTROL_TIMESTEP_ms();
  calculateCurrentError();
  calculateAccumulatedError();
  calculateDeltaError();
  printStatus();
  updateMotor();
  checkForPiCommand();
 }

