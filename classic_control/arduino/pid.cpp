/*

This little arduino script is to tune and play with the parameters
of the motorControl PID

*/

#include <Adafruit_Sensor.h>            // IMU
#include <Adafruit_BNO055.h>            // IMU
#include <Wire.h>                       // I2C
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
imu::Quaternion quat;
imu::Vector<3> axis;
float rawTheta() {
  quat = bno.getQuat();
  axis = quat.toEuler();
  return axis.z();
}

// calculates and stores
// theta and thetaDot
void updateTheta(){
  theta = rawTheta();
  thetaDot = (theta - lastTheta); // no division by dt because only called once per loop
  lastTheta = theta;
}

//------------------------------------------------


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

  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);
  Serial.println("Setting up interrupts...Done!");

  wheels.updateRadsPerSec(0);
  wheels.updatePids(0.015,0,0.05);

  thetaPid.updateSetpoint(0);
}

float newRadPerSec;
void printStuff(float dt){
  String foo = String(dt) + ", " + String(theta) + ", " + newRadPerSec + ", " + thetaPid.getkp();
  Serial.println(foo);
}

long innerDt;
void loop(){
  innerDt = innerWaiter.wait();
  wheels.spin(innerDt);
  if(outerWaiter.isTime()){
    long dt = outerWaiter.starting();
    updateTheta();
    float newRadPerSec = thetaPid.generateCommand(theta, dt);
    wheels.updateRadsPerSec(newRadPerSec);
    checkForPiCommand();
    printStuff(dt);
  }

}
