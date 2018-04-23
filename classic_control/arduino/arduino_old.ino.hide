// CLASSIC CONTROL -- PID

#include <StandardCplusplus.h>
#include <sstream>      // std::istringstream

#include <cmath>

#include "../../cpp_lib/constants.cpp"
#include "../../cpp_lib/servoMotor.cpp"
#include "../../cpp_lib/state.cpp"
#include "../../cpp_lib/pid.cpp"

ServoMotor motorLeft(LH_ENCODER_A,LH_ENCODER_B, 1);
ServoMotor motorRight(RH_ENCODER_A,RH_ENCODER_B, -1);

State state(motorLeft, motorRight);

// motor control (inner) PID
Pid motorPid(MOTOR_CONTROL_TIMESTEP);

// position control (outer) PID
Pid positionPid(POSITION_CONTROL_TIMESTEP);

float motorCommand = 0; // holds the final command to motor

// following variables used to control the loop times
long positionTimeMarker = 0;
long motorTimeMarker = 0;
long lastPrintNow = 0; // only used to verify final loop time

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

void generateMotorCommand(){

}

void generatePositionCommand(){
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

  // these should not need to change
  motorPid.updateParameters(0.1, 0.1, 0.1);

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

  Serial.print("Initializing state and IMU...");
  if(state.initialize()){
    Serial.println("Done!");
  }else{
    Serial.println("Inertial Sensor failed, or not present. Halting.");
    while(true){}
  }

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
  updateState(motor_delta, MOTOR_CONTROL_TIMESTEP);
  generateMotorCommand();
  wait_POSITION_CONTROL_TIMESTEP_ms();
  calculateCurrentError();
  calculateAccumulatedError();
  calculateDeltaError();
  printStatus();
  updateMotor();
  checkForPiCommand();
 }

