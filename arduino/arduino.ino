#include <StandardCplusplus.h>
#include <iostream>
#include <string>
#include <iomanip> // setprecision
#include <sstream> // stringstream
#include <Adafruit_Sensor.h> // IMU
#include <Adafruit_BNO055.h> // IMU
#include <Wire.h> // I2C
#include <cmath>
#include <SPI.h>  // radio controls
#include "RF24.h" // radio controls
#include "servoMotor.cpp"
#include "constants.cpp"

ServoMotor motorLeft(LH_ENCODER_A,LH_ENCODER_B, 1);
ServoMotor motorRight(RH_ENCODER_A,RH_ENCODER_B, -1);

RF24 radio(RADIO_CE, RADIO_CSN);

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55, 40);

float dt = 0;
float xPos = 0; // meters
float lastXPos = 0;
float xVel = 0; // meters / sec
float phi = 0; // radians
float phiDot = 0; // radians / sec
float lastPhi = 0;
float theta = 0; // radians
float thetaDot = 0; // radians / sec
float lastTheta = 0;
long timeDelta = 0;
long timeMarker = 0;
long nowish = 0;
float previousGain = 0;

// vars to hold incoming serial messages from Pi
char tmp_char;
std::string str;

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
  beep(150, 150);
  beep(150, 75);
  beep(75, 75);
  beep(150, 150);
  beep(250, 250);
  beep(150, 150);
  beep(250, 150);
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

void turnIndicatorLightOff(){
  digitalWrite(INDICATOR, LOW);
}

void turnIndicatorLightOn(){
  digitalWrite(INDICATOR, HIGH);
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
  thetaDot = (theta - lastTheta) / dt;
  lastTheta = theta;
}

// calculates and stores
// x and xDot (Meters and Meters/Sec)
//
// argument dt is in seconds
void fetchX(float dt){
  xPos = rawX();
  xVel = (xPos - lastXPos) / dt;
  lastXPos = xPos;
}

// calculates and stores
// phi and phiDot
//
// argument dt is in seconds
void fetchPhi(float dt){
  phi = rawPhi();
  phiDot = (phi - lastPhi) / dt;
  lastPhi = phi;
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

void processMotorCommand(std::string message){
  float power = std::atof(message.c_str());
  updatePower(power);
}

void processResetEncoderCommand(){
  Serial.println("resetting encoders");
  motorRight.resetCount();
  motorLeft.resetCount();
}

void sendCommandToWinch(char* command){
  radio.stopListening();
  Serial.print("Winch command: ");
  Serial.println(command);

  for(int i=0;i<10;i++){
    if(radio.write(command, 1)){
      Serial.println("Sent command!");
      radio.startListening();
      return;
    }
    Serial.println("Failed. Trying again...");
    delay(500);
  }
  Serial.println("Failed. Giving up :(");
  radio.startListening();
}

// lazy: the only thing the winch EVER sends
// back today is if its done with resettting.
// so if we receive ANYTHING (today) we assume
// thats what it is.
bool hasWinchResponded(){
  if ( radio.available() ){
    int msg[1];

    radio.read(msg, 1);
    return true;
  } else {
    return false;
  }
}

void sendToPi(char code, String message){
  String final;
  final = String(code) + message + "!";
  Serial.print("sending to Pi: ");
  Serial.println(final.c_str());
  Serial2.write(final.c_str());
}

void clearRadioBuffer(){
  while(hasWinchResponded()){;}
}

// Blocking. Issues reset command to winch and waits for winch to finish.
void processResetRobotCommand(){
  // motors should already be off but
  // just in case, since this method blocks
  updatePower(0);
  clearRadioBuffer();
  bool isFinished = false;
  Serial.println("Requesting Mr. Winch to reset me.");
  // double loop needed in case there is a radio blip
  while(true){
    sendCommandToWinch((char *)"L");
    Serial.print("_");
    for(int i = 0; i < 30; i++)
    {
      delay(500);
      Serial.print(".");
      if(hasWinchResponded()){ isFinished = true; break; }
    }
    if(isFinished) { break; }
    delay(1000);
  }
  Serial.println("Done!");
  String message = String("t");
  sendToPi('W', message);
}

void sendStateToPi(){
  String message = String(xPos, 4) + "," + String(xVel, 4) + "," + String(theta, 4) + "," + String(thetaDot, 4);
  sendToPi('S', message);
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
    case 'M': processMotorCommand(message.substr(1));
              break;
    case 'R': processResetEncoderCommand();
              break;
    case 'L': processResetRobotCommand(); // lift (reset) robot. Blocking.
              break;
    case 'S': sendCommandToWinch((char *)"S"); // give robot slack. Non-blocking.
              break;
    case 'O': sendStateToPi();
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

void setup() {
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

  // radio setup
  Serial.print("Initializing radio...");
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setRetries(15,15);
  radio.setPayloadSize(4);
  byte addresses[][6] = {"1Node","2Node"};
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);
  Serial.println("Done!");

  // Turn on indicator light because we are ready to rock.
  Serial.println("Done Initializing!");
  updatePower(0); // ensure motors are off
  turnIndicatorLightOn();

  // play init song
  playInitSong();

  // print out two lines in case there  was garbage (noise) in serial setup
  Serial.println("\n");
  delay(100);

  // ensure our timeDelta is accurate by resetting timeMarker
  // just before the loop starts
  timeMarker = millis();
}

void loop() {
  nowish = millis();
  timeDelta = nowish - timeMarker;
  if(timeDelta < TIMESTEP){return;}
  timeMarker = nowish;
  dt = (float) timeDelta / 1000; // Units are floats in seconds.
  fetchTheta(dt);
  fetchX(dt);
  sanityCheck();
  checkForPiCommand();
}
