#include "arduino.h"

class ESPCOPTER{
  
  public:
  void espcopterSetup();
  void redLed_Digital(int OnOff);
  void blueLed_Digital(int OnOff);
  void greenLed_Digital(int OnOff);

  void redLed_Digital(boolean OnOff);
  void blueLed_Digital(boolean OnOff);
  void greenLed_Digital(boolean OnOff);
/*
  void redLed_Analog(int value);
  void blueLed_Analog(int value);
  void greenLed_Analog(int value);
*/
  void buzzer(boolean OnOff, int timer);

  void motorFL_Analog(int value);
  void motorFR_Analog(int value);
  void motorRL_Analog(int value);
  void motorRR_Analog(int value);
  
  
  #define blueLed_ 0 // 
  #define redLed_ 2
  #define greenLed_ 16 //
  private:
  unsigned long previousMillisBuzzer = 0;
  boolean buzzerState = LOW;
  
  };
