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
#include "../cpp_lib/constants.cpp"     // yeah. Constants.
#include "../cpp_lib/wheels.cpp"        // control get raw encoder state
#include "../cpp_lib/state.cpp"         // holds all final processed state
#include "../cpp_lib/pid.cpp"           // encapsulates PID functionality
#include "../cpp_lib/waiter.cpp"        // waiter helper to help with...waiting

Wheels wheels;
State state(POSITION_CONTROL_TIMESTEP, &wheels);
Pid motorPid(POSITION_CONTROL_TIMESTEP);
Waiter waiter(POSITION_CONTROL_TIMESTEP);

void leftEncoderEvent(){ wheels.leftEncoderEvent(); }
void rightEncoderEvent(){ wheels.rightEncoderEvent(); }

void printStatus(int dt, float command){
  String results = String(state.getPhiDot(), 5);
  results += ", " + String(command,5);
  Serial.println(results);
}

//---------------------------------------
// move all this pi-communication stuff to
// its own lib

// vars to hold incoming serial messages from Pi
char tmp_char;
std::string str;

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


  motorPid.updateParameters(values[0], values[1], values[2]);
}

void updateSetpoint(std::string message){
  Serial.print("Updating Setpoint to ");
  Serial.println(message.c_str());
  float newSetpoint = String(message.c_str()).toFloat();
  motorPid.updateSetpoint(newSetpoint);
}

void handle_command_from_pi(std::string message){
  Serial.print("got the message from Pi: ");
  Serial.println(message.c_str());
  char command;
  command = message[0];
  switch (command){
    case 'K': updatePIDValues(message.substr(1));
              break;
    case 'S': updateSetpoint(message.substr(1));
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

  if(state.initialize()){
    Serial.println("initialized state!");
  }else{
    Serial.println("failed to initialize state :(");
  }

  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);
  Serial.println("Setting up interrupts...Done!");

  motorPid.updateParameters(0.001,0,0); // P, I, D
  motorPid.updateSetpoint(6.283);  // rads/sec rotational velocity of wheels (60rpm)
  motorPid.updateSetpoint(6.283 * 2);
  wheels.command(0);
}

float lastCommand;
void loop() {
  int dt = waiter.wait();
  state.updateState(dt);
  float commandDelta = motorPid.generateCommand(state.getPhiDot(), dt);
  float newCommand = wheels.getMotorCommand() + commandDelta;
  wheels.command(newCommand);
  printStatus(dt, newCommand);
  long lmec = wheels.getLeftMotorEdgeCount();
  long rmec = wheels.getRightMotorEdgeCount();
  String foo = "EC: " + String(lmec) + ", " + String(rmec);
  Serial.println(foo);
}
