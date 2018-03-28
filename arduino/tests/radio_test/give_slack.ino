#include <StandardCplusplus.h>
#include <iostream>
#include <string>
#include <iomanip> // setprecision
#include <sstream> // stringstream
#include <Wire.h>
#include <cmath>
#include <SPI.h>
#include "RF24.h" // radio controls


#include "../../constants.cpp"


RF24 radio(RADIO_CE, RADIO_CSN);

std::string waitingForWinchToFinishLifting = "f";

void turnIndicatorLightOff(){
  digitalWrite(INDICATOR, LOW);
}

void turnIndicatorLightOn(){
  digitalWrite(INDICATOR, HIGH);
}

void sendCommandToWinch(char* command){
  radio.stopListening();

  if (radio.write(command, 1)){
    Serial.print(F("Sent "));
    Serial.println(command);
  } else {
    Serial.println(F("failed"));
  }

  radio.startListening();
}

void setup() {
  // indicator pin is an output for LED
  pinMode(INDICATOR, OUTPUT);
  // turn off indicator light while we setup
  turnIndicatorLightOff();

  // Open serial communications
  // and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {;}
  Serial.println("Beginning Setup...");

  // radio setup
  Serial.println("Initializing radio...");
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);

  byte addresses[][6] = {"1Node","2Node"};
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1,addresses[1]);

  // Turn on indicator light because we are ready to rock.
  Serial.println("Done Initializing!");
  turnIndicatorLightOn();


  sendCommandToWinch((char *)"S");
}

void loop() {
  delay(1000);
}
