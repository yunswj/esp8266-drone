#include "block.h"


int BLOCK::GetRoll(){
 return roll;
}
int BLOCK::GetPitch(){
 return pitch;
}
int BLOCK::GetYaw(){
 return yaw;
}

int BLOCK::GetOptRoll(){
 return rollOpt;
}
int BLOCK::GetOptPitch(){
 return pitchOpt;
}


int BLOCK::GetTurn(){
 return turnYaw;
}
void BLOCK::turnLeft(int32_t value){
turnYaw = turnYaw + value;
}
void BLOCK::turnRight(int32_t value){
turnYaw =  turnYaw - value;
}
void BLOCK::goForward(int32_t timing){
preTimeBlock = millis();
delayTime = timing;
newAction = true;
  rollOpt = 0;
  pitchOpt = -actionDegree;
 roll = 0;
 pitch = 0;
 yaw = 0;
}
void BLOCK::goBack(int32_t timing){
preTimeBlock = millis();
delayTime = timing;
newAction = true;
  rollOpt = 0;
  pitchOpt = actionDegree;
 roll = 0;
 pitch = 0;
 yaw = 0;
}

void BLOCK::goLeft(int32_t timing){
preTimeBlock = millis();
delayTime = timing; 
newAction = true;
  rollOpt = actionDegree;
  pitchOpt = 0;
 roll = 0;
 pitch = 0;
 yaw = 0;
}

void BLOCK::goRight(int32_t timing){
preTimeBlock = millis();
delayTime = timing;  
newAction = true;
  rollOpt = -actionDegree;
  pitchOpt = 0;
  roll = 0;
  pitch = 0;
  yaw = 0;
}

void BLOCK::blocklyLoop(){

   currentTimeBlock = millis();
   if(currentTimeBlock - preTimeBlock > delayTime && newAction == true){
   preReverseTimeBlock = currentTimeBlock;
   delayTime_ = delayTime;
   roll = 4*rollOpt;
   pitch = 4*pitchOpt;
   yaw = 0;
   rollOpt = 0;
   pitchOpt = 0;
   newAction = false;
   reverseAction = true;
   }

   if(currentTimeBlock - preReverseTimeBlock > round(delayTime_/4) && reverseAction == true ){
    roll = 0;// 
    pitch = 0;//  
    reverseAction = false;
   }
}
