#include "arduino.h"

class BLOCK{
  public:
   void goForward(int32_t timing);
    void goBack(int32_t timing);
     void goLeft(int32_t timing);
      void goRight(int32_t timing);
      
      void turnLeft(int32_t timing);
      void turnRight(int32_t timing);
      
       void blocklyLoop();
       int GetRoll();
       int GetPitch();
       int GetYaw();
       int GetOptRoll();
       int GetOptPitch();
         int GetTurn();
       private:
       int firstTime = 1;
       int delayTime = 0;
        int delayTime_ = 0;
        
        int roll = 0;
         int pitch = 0;
          int yaw = 0;

         int roll_ = 0;
         int pitch_ = 0;
         int yaw_ = 0;

         int turnYaw = 0;
     

         int rollOpt = 0;
         int pitchOpt = 0;

          int actionDegree = 15;
         boolean  reverseAction = false;
          boolean newAction = false;
       unsigned long preTimeBlock = 0;
       unsigned long currentTimeBlock = 0;
        unsigned long preReverseTimeBlock = 0;
};
