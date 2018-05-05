#ifndef __BEAKER_OUTPUTS__
#define __BEAKER_OUTPUTS__

class Outputs{
  public:

  static void turnIndicatorLightOff(){
    digitalWrite(INDICATOR, LOW);
  }

  static void turnIndicatorLightOn(){
    digitalWrite(INDICATOR, HIGH);
  }

  static void turnBuzzerOn(){
    digitalWrite(BUZZER_PIN, HIGH);
  }

  static void turnBuzzerOff(){
    digitalWrite(BUZZER_PIN, LOW);
  }

  static void beep(int durationOn, int durationOff){
    turnBuzzerOn();
    delay(durationOn);
    turnBuzzerOff();
    delay(durationOff);
  }

  static void sayHi(){
    beep(100,100);
    beep(100,100);
  }

  // since Arduino does not have an OS outside of the application, when
  // the app really crashes, and we don't want it to reboot, then call
  // this method.
  // Consider adding an OLED so Beaker can tell you exactly why you are here.
  static void permaError(){
    while(true){
      beep(500,500);
    }
  }

  static void init(){
    // indicator pin is an output for LED
    pinMode(INDICATOR, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    // say hi to the world
    turnIndicatorLightOn();
    sayHi();
  }
};

#endif
