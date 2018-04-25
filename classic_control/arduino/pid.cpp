/*

This little arduino script is to tune and play with the parameters
of the motorControl PID

*/

#include <Adafruit_Sensor.h>            // IMU
#include <Adafruit_BNO055.h>            // IMU
#include <Wire.h>                       // I2C
#include <SPI.h>          // for sending telemetry over radio
#include "RF24.h"         // for sending telemetry over radio
#include <StandardCplusplus.h>
#include <stdlib.h>
#include <string>
#include <sstream> // stringstream
#include "../../cpp_lib/constants.cpp"     // yeah. Constants.
#include "../../cpp_lib/waiter.cpp"        // waiter helper to help with...waiting
#include "../../cpp_lib/pid.cpp"        // waiter helper to help with...waiting
#include "../../cpp_lib/wheels.cpp"        // control get raw encoder state

Wheels wheels;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 40);
Waiter outerWaiter(POSITION_CONTROL_TIMESTEP);
Waiter innerWaiter(MOTOR_CONTROL_TIMESTEP);
Pid thetaPid(POSITION_CONTROL_TIMESTEP);
void leftEncoderEvent(){ wheels.leftEncoderEvent(); }
void rightEncoderEvent(){ wheels.rightEncoderEvent(); }

// bunch of variables for timing...
float theta, lastTheta, thetaDot;

// control values
int kTheta, kThetaDot, kPhi, kPhiDot;

// allows for minor variations on where "Balanced" really is.
float thetaBias = 0;

float newRadPerSec;

// PINS 6, 7 for radio ce / csn
RF24 radio(6,7);

//-------------------------------
// Comm w Pi. Can this be in a lib?

// vars to hold incoming serial messages from Pi
char tmp_char;
std::string str;

void updateMotorPIDValues(std::string message){
  Serial.println("Updating Motor PID Values!!!");
  Serial.println(message.c_str());

  float values[3];
  std::istringstream ss( message );
  std::copy(
    std::istream_iterator <float> ( ss ),
    std::istream_iterator <float> (),
    values
    );


  wheels.updatePids(values[0], values[1], values[2]);
}

void updateRadsPerSec(std::string message){
  Serial.print("Updating Setpoint to ");
  Serial.println(message.c_str());
  float newSetpoint = String(message.c_str()).toFloat();
  wheels.updateRadsPerSec(newSetpoint);
}

void updateThetaBias(std::string message){
  Serial.print("Updating Theta Bias to ");
  Serial.println(message.c_str());
  thetaBias = String(message.c_str()).toFloat();
}

void updateThetaPIDValues(std::string message){
  Serial.println("Updating Theta PID Values!!!");
  Serial.println(message.c_str());

  float values[3];
  std::istringstream ss( message );
  std::copy(
    std::istream_iterator <float> ( ss ),
    std::istream_iterator <float> (),
    values
    );


  thetaPid.updateParameters(values[0], values[1], values[2]);
}

void updateControlValues(std::string message){
  Serial.println("Updating Control Values!!!");
  Serial.println(message.c_str());

  float values[4];
  std::istringstream ss( message );
  std::copy(
    std::istream_iterator <float> ( ss ),
    std::istream_iterator <float> (),
    values
    );


  kTheta = values[0];
  kThetaDot = values[1];
  kPhi = values[2];
  kPhiDot = values[3];
}

void handle_command_from_pi(std::string message){
  Serial.print("got the message from Pi: ");
  Serial.println(message.c_str());
  char command;
  command = message[0];
  switch (command){
    case 'K': updateMotorPIDValues(message.substr(1));
              break;
    case 'S': updateRadsPerSec(message.substr(1));
              break;
    case 'P': updateThetaPIDValues(message.substr(1));
              break;
    case 'B': updateThetaBias(message.substr(1));
              break;
    case 'C': updateControlValues(message.substr(1));
              break;
  }
}

// update data from Pi.
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

// in radians
float rawTheta() {
  imu::Vector<3> eulerDegrees = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return 0.0174533 * eulerDegrees.z();
}

// calculates and stores theta and thetaDot
//
// rads and rads/sec
void updateTheta(){
  theta = rawTheta() + thetaBias;
  thetaDot = (theta - lastTheta);
  lastTheta = theta;
 }

float kThetaTerm(){ return theta * kTheta * -1; }
float kThetaDotTerm() { return thetaDot * kThetaDot * -1; }
float kPhiTerm() { return wheels.getPhi() * kPhi; }
float kPhiDotTerm() { return wheels.getPhiDot() * kPhiDot; }

void computeNewRadsPerSec(long dt){
  newRadPerSec = kThetaTerm();
  newRadPerSec += kThetaDotTerm();
  newRadPerSec += kPhiTerm();
  newRadPerSec += kPhiDotTerm();
}

void printStuff(float dt){
  String messageOne = String(dt);
  messageOne += "," + String(theta,4);
  messageOne += "," + String(thetaDot,4);
  messageOne += "," + String(wheels.getPhi(),4);

  String messageTwo = "," + String(wheels.getPhiDot(),4);
  messageTwo = "," + String(kTheta);
  messageTwo += "," + String(kThetaDot);
  messageTwo += "," + String(kPhi);
  messageTwo += "," + String(kPhiDot);
  messageTwo += ";"; // end of line character (natch)

  char packetOne[32];
  char packetTwo[32];

  messageOne.toCharArray(packetOne, 32);
  messageTwo.toCharArray(packetTwo, 32);

  radio.write(packetOne, sizeof(packetOne));
  radio.write(packetTwo, sizeof(packetTwo));
}

void setup() {
  // Open console serial communications
  Serial.begin(115200);
  while (!Serial) {;}
  Serial.println("\n\nBeginning initializations...");

  // Open serial comm to talk to RaspPi.
  Serial.print("Initializing comm to Rasp Pi...");
  Serial2.begin(115200);
  while (!Serial2) {;}
  Serial.println("Done!");

  //Check to see if the Inertial Sensor is wired correctly and functioning normally
  Serial.print("Initializing IMU...");
  if (!bno.begin(bno.OPERATION_MODE_IMUPLUS)) {
    Serial.println("Inertial Sensor failed, or not present");
    while(true) {;}
  }
  bno.setExtCrystalUse(true);
  Serial.println("Done!");

  // radio setup
  Serial.print("Initializing Radio....");
  const byte address[6] = "00001";
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  Serial.println("Done!");

  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);
  Serial.println("Setting up interrupts...Done!");

  wheels.updateRadsPerSec(0);
  wheels.updatePids(0.015,0,0.05);

  thetaBias = THETA_OFFSET;

  thetaPid.updateSetpoint(0);
  kTheta = kThetaDot = kPhi = kPhiDot = 0;
}

void loop(){
  long innerDt = innerWaiter.wait();
  wheels.spin(innerDt);
  if(outerWaiter.isTime()){
    long dt = outerWaiter.starting();
    updateTheta();
    // newRadPerSec = thetaPid.generateCommand(theta, dt);
    computeNewRadsPerSec(dt);
    wheels.updateRadsPerSec(newRadPerSec);
    checkForPiCommand();
    printStuff(dt);
  }
}
