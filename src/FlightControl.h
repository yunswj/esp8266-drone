#include "Parameter.h"

// Includes of Expressif SDK

extern "C"{
  #include "pwm.h"
  #include "user_interface.h"
}

////// PWM

// Period of PWM frequency -> default of SDK: 5000 -> * 200ns ^= 1 kHz

#define PWM_PERIOD  256  //256 - 500 - 1024


// PWM channels

#define PWM_CHANNELS 4

// PWM setup (choice all pins that you use PWM)

uint32 io_info[PWM_CHANNELS][3] = {
  // MUX, FUNC, PIN
  //  {PERIPHS_IO_MUX_GPIO5_U, FUNC_GPIO5,   5}, // D1
  //  {PERIPHS_IO_MUX_GPIO4_U, FUNC_GPIO4,   4}, // D2
  //  {PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0,   0}, // D3
  //  {PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2,   2}, // D4
  {PERIPHS_IO_MUX_MTMS_U,  FUNC_GPIO14, 14}, // D0
  {PERIPHS_IO_MUX_MTDI_U,  FUNC_GPIO12, 12}, // D1
  {PERIPHS_IO_MUX_MTCK_U,  FUNC_GPIO13, 13}, // D2
  {PERIPHS_IO_MUX_MTDO_U,  FUNC_GPIO15 ,15}, // D3
                         // D0 - not have PWM :-(
};

// PWM initial duty: all off

uint32 pwm_duty_init[PWM_CHANNELS];

// Dimmer variables


unsigned long ts=millis();
unsigned long tsbat=millis();

unsigned long tsOpt =millis();

boolean ledState =   HIGH;

float factor;
float factor_;

int stopFlightHand= 0;

float innerEffect;

int throttleMotorChecker =0;
  
void FlightControl(){

if((millis()-tsbat)>1000){  //Update only on 
 tsbat = millis();
 //Serial.println(analogRead(A0)*5.5);
 if(analogRead(A0)*5.5  < 2800){
  batteryCount= batteryCount+1;
 }else{
 batteryCount= 0;
}
if(batteryCount > 5){
  landingOff = 0;
  esp.greenLed_Digital(false);
  esp.blueLed_Digital(true);
}
}
 
if((millis()-ts)>1){  //Update only once per 2ms (500hz update rate)
  ts = millis();
   
  ahrs.compute(attitude, rate,attitudeRadian,rateRadian);  
  ahrs.headingMag(rateRadian, attitudeRadian, degree , throttle); 

  if( yawControl==0){
  degree[0] = 0;
  }
    

  if( attitudeRadian[0] >  0.6  || attitudeRadian[0] <  -0.6 ||   attitudeRadian[1] > 0.6 ||  attitudeRadian[1] < -0.6  ){
    stopFlightHand++;
  }else{
    stopFlightHand= 0;
    stopFlightControl = 1;
  }
  
  if( stopFlightHand > 50){
   stopFlightControl = 0;
  }

   
   
   #ifdef otoMission
   loop_();
   #endif

   #ifdef BLOCKLY
   blockly.blocklyLoop();
   SetPointOptBlock[0] = blockly.GetOptRoll();
   SetPointOptBlock[1] = blockly.GetOptPitch();
   SetPointOptMain[0] = blockly.GetRoll();
   SetPointOptMain[1] = blockly.GetPitch();
  // yaw_Control = blockly.GetTurn();
   #endif


   if(opticalFlowControl == 1 && autoOpt == 1){
   SetPointOptRemote[0] =  -map(RX_roll, -100, 100, -15, 15);
   SetPointOptRemote[1] =  -map(RX_pitch, -100, 100, -15, 15);
   SetPoint[0] =0;
   SetPoint[1] =0;
   }

   SetPointOpt[0] =  SetPointOptBlock[0] + SetPointOptRemote[0];
   SetPointOpt[1] = SetPointOptBlock[1] + SetPointOptRemote[1] ;

   if(autoOpt == 1  ){  // 3 second whole control zero
   if( _autoOpt !=autoOpt){
   tsOpt = millis();
   }
   if( 5000 >  millis() - tsOpt ){
   SetPointOpt[0] = 0;
   SetPointOpt[1] = 0;
   RX_roll_Multi = 0;
   RX_pitch_Multi = 0;
   SetPoint[0] =0;
   SetPoint[1] =0;
   SetPoint[2] =0;
   }

   innerEffect=1;
   
   /*
   if( 2000 >  millis() - tsOpt ){//??
   innerEffect = map(throttle, 300 , 400, 0,1 );
   }else{
   innerEffect=1;
   }*/
   
   }
   _autoOpt = autoOpt;
   

   fly();
   
  
   #ifdef vl53l0x
   if(vl5310xControl == 1){
   vl5310x();
   }
   #endif
    
   #ifdef opticalFlow
   if(opticalFlowControl == 1){
   opticalSensor();
   }
   #endif  
   
   #ifdef MULTI_RANGER
   if(multiRangerControl == 1){
   multiRangerLoop();
   }
   #endif 


 
   if(throttle > 2){ 
   roll.compute( 0 , SetPoint[0] + SetPointOptMain[0],throttle, -xOpt.output*innerEffect  +  Trim_Roll +Trim_Roll_Bs - xMulti.output + xMultiM.output  ,-attitude[1], rate[1]); 
   pitch.compute( 1 , SetPoint[1] + SetPointOptMain[1] ,throttle, -yOpt.output*innerEffect +  Trim_Pitch+  Trim_Pitch_Bs - yMulti.output  + yMultiM.output  , -attitude[0], -rate[0]);
   yaw.compute( 2 , SetPoint[2]   , throttle,Trim_Yaw, (degree[0]+ yaw_Control + autoYawControl)*100, -rate[2]);
   }else{
   ahrs.setZero();  
   }
  }

  if(throttle >= motorMax) {throttle = motorMax;} else if(throttle <= 0) {throttle = 0;}
  
  factor = 1  + throttle/100;//100 82 //+pow(2,throttle/225);// 1 + pow(2,throttle/225);
  factor_ = 100;
  motorFL = throttle + (roll.output/factor_)*factor - (pitch.output/factor_)*factor + (yaw.output/factor_)*factor ;//+ (xOpt.GetItermRateBase()/25);// + (yOpt.GetItermRateBase()/25); // pitch
  motorFR = throttle - (roll.output/factor_)*factor - (pitch.output/factor_)*factor - (yaw.output/factor_)*factor ;//- (xOpt.GetItermRateBase()/25)+100;// + (yOpt.GetItermRateBase()/25); //rool
  motorRL = throttle + (roll.output/factor_)*factor + (pitch.output/factor_)*factor - (yaw.output/factor_)*factor ;//+ (xOpt.GetItermRateBase()/25);// - (yOpt.GetItermRateBase()/25); // pitch
  motorRR = throttle - (roll.output/factor_)*factor + (pitch.output/factor_)*factor + (yaw.output/factor_)*factor ;//- (xOpt.GetItermRateBase()/25)+100;// - (yOpt.GetItermRateBase()/25); //rool


  
  int16_t motorFL_ = round(map(motorFL*(1 -pitch_Extra +roll_Extra),0,1023,0,PWM_PERIOD)); //255
  int16_t motorFR_ = round(map(motorFR*(1 -pitch_Extra -roll_Extra),0,1023,0,PWM_PERIOD));
  int16_t motorRL_ = round(map(motorRL*(1 +pitch_Extra +roll_Extra),0,1023,0,PWM_PERIOD));
  int16_t motorRR_ = round(map(motorRR*(1 +pitch_Extra -roll_Extra),0,1023,0,PWM_PERIOD));

  motorFL_ = constrain(motorFL_,0,PWM_PERIOD);
   motorFR_ = constrain(motorFR_,0,PWM_PERIOD);
    motorRL_ = constrain(motorRL_,0,PWM_PERIOD);
     motorRR_ = constrain(motorRR_,0,PWM_PERIOD);

  // Input Control Value to PWM Pins

  if(throttle > 2){    
  pwm_set_duty((motorFL_), 0);
  pwm_set_duty((motorFR_), 3);
  pwm_set_duty((motorRR_), 2);
  pwm_set_duty((motorRL_), 1);
   
  }else if(armControl == 0 || throttleMotorChecker == 1 ) {
  pwm_set_duty(0, 0);
  pwm_set_duty(0, 3);
  pwm_set_duty(0, 2);
  pwm_set_duty(0, 1);
  }
  pwm_start();  

}


void modeControl(){
if(armControl == 1 && landingOff == 1 && stopFlightControl == 1){
  
    throttleControl = 1;
    if(flyMode_1 == 1){
    yawControl=1;
    }else{
    yawControl=0; 
    }
 
    if(flyMode_2 == 1){
    throttleControl = 0;
    otoHover=1;
    }else{
    otoHover=0;
    }
    
    if(flyMode_3 == 1){
    throttleControl = 0;
    autoOpt =1;
    otoHover=1;
    }else if ( flyMode_2 != 1){
    autoOpt =0;
    otoHover=0;
    //throttleControl = 0;
    }else{
    autoOpt =0;
    //throttleControl = 0;
    }

   
    
    }else if(landingOff == 0){
    targetOto = 1; // 1
    if( otoMeasure < 150){
    throttleControl = 0;
    throttle = throttle-1;
    yawControl=0;
    otoHover=0;
    autoOpt =0;
    if(throttle < 200){
    throttle=0;
    landingOff=1;
    targetOto = targetOtoDf;
    flyMode_1 =0;
    flyMode_2 =0;
    flyMode_3 =0;
    //missionCounter = -1;
    //takeMissions = 1;
    }
    }

    
    }else if(stopFlightControl == 0){
    armControl=0;
    throttleControl = 0;
    throttle=0;
    otoHover=0;
    autoOpt =0;
    yawControl=0;
    }else{
    throttleControl = 0;
    otoHover=0;
    autoOpt =0;
    yawControl=0;
    throttle=0;
    
    //missionCounter = -1;
    //takeMissions = 1;
    
    }

    if(throttleControl == 1 ){
    if( throttle < RX_throttle){
    throttle = throttle +2.5;
    }
    if( throttle > RX_throttle){
    throttle = throttle -2.5;
    }
    }
    

   if( buzzerControl == 1){
   esp.buzzer(true, 50);
   }else{
   esp.buzzer(false, 50);
   }
  

    
   SetPoint[0] = map(RX_roll, -100, 100, -ROLL_LIMIT, ROLL_LIMIT);
   SetPoint[1] = map(RX_pitch, -100, 100, -PITCH_LIMIT, PITCH_LIMIT);


    if(armControl == 0 ){
    targetOto =  500;
    }
    

   if(yawControl == 0 ){
   SetPoint[2] = map(RX_yaw, -100, 100, -YAW_LIMIT, YAW_LIMIT);  
   }else{
   SetPoint[2] = 0;  //??
   yaw_Control = -map(RX_yaw, -100, 100, -180, 180);
   }

  throttle = constrain(throttle,0, motorMax);

  if(  2 < throttle ){
    throttleMotorChecker =1;
  }


  
}




void setMotorSpeedFL(int speedMotor){
  pwm_set_duty((speedMotor), 0);
  pwm_start();  
}

void setMotorSpeedFR(int speedMotor){
  pwm_set_duty((speedMotor), 3);
  pwm_start();  
}

void setMotorSpeedRR(int speedMotor){
  pwm_set_duty((speedMotor), 2);
  pwm_start();  
}

void setMotorSpeedRL(int speedMotor){
  pwm_set_duty((speedMotor), 1);
  pwm_start();  
}


float getMpuAccelX(){
  return  ahrs.getAccelX();
}

float getMpuAccelY(){
  return  ahrs.getAccelY();
}

float getMpuAccelZ(){
  return  ahrs.getAccelZ();
}

float getMpuGyroX(){
   return  ahrs.getGyroX();
}

float getMpuGyroY(){
  return  ahrs.getGyroY();
}
float getMpuGyroZ(){
  return  ahrs.getGyroZ();
}


float getMpuAttitudeX(){
 return attitude[0];
}

float getMpuAttitudeY(){
 return attitude[1];
}
float getMpuAttitudeZ(){
 return attitude[2];
}

float getMpuRateX(){
 return rate[0];
}

float getMpuRateY(){
 return rate[1];
}

float getMpuTemp(){
 return ((float) ahrs.readTemp()) / 333.87f + 21.0f;
}

void setTrimRoll(int x){
  Trim_Roll = x;
}

void setTrimPitch(int x){
  Trim_Pitch = x;
}
void setTrimYaw(int x){
  Trim_Yaw = x;
 //yaw.SetItermRate(x);
}


void setArmControl(boolean x){
  if( x == true){
  armControl = 1;
  }else{
  armControl = 0;
  }
}

void setFlyMode_1(boolean x){
  if( x == true){
  flyMode_1 = 1;
  }else{
  flyMode_1 = 0;
  }
}


void setFlyMode_2(boolean x){
  if( x == true){
  flyMode_2 = 1;
  }else{
  flyMode_2 = 0;
  }
}

void setFlyMode_3(boolean x){
  if( x == true){
  flyMode_3 = 1;
  }else{
  flyMode_3 = 0;
  }
}

void landing(boolean x){
  if( x == true){
  landingOff = 0;
  }else{
  landingOff = 1;
  }
}

void setMotorMax(int x){
  motorMax = x;
}

int getRX_throttle(){
return round(RX_throttle);
}

int getRX_roll(){
return round(RX_throttle);
}

int getRX_pitch(){
return round(RX_pitch);
}

int getRX_yaw(){
return round(RX_yaw);
}

void setVl5310xControl(boolean x){
  if( x == true){
  vl5310xControl = 1;
  }else{
  vl5310xControl = 0;
  }
}


void setTargetOto(int x){
targetOto = x;
}

int getOtoMeasure(){
  return otoMeasure;
}


void SetOptPoint_X(int x){
SetPointOpt[0] = x;
}
void SetOptPoint_Y(int x){
SetPointOpt[1] = x;
}

int getOptData_X(){
  return deltaX;
}

int getOptData_Y(){
  return deltaY;
}


int getFilterOptData_X(){
  return deltaCalX + deltaCalXAnt;
}

int getFilterOptData_Y(){
  return deltaCalY + deltaCalYAnt;
}

int getBatteryVoltage(){
  return round(analogRead(A0)*5.5) ;
}

int getBatteryLevel(){
  return map(analogRead(A0)*5.5,3300,4200,0,100) ;
}


int Distance_Y_1(){
  return DistanceY1;
}
int Distance_Y_0(){
  return DistanceY0;
}
int Distance_X_1(){
  return DistanceX1;
}
int Distance_X_0(){
  return DistanceX0;
}

String getSfVersion(){
  return sfVersion;
}

String getSpVersion(){
  return spVersion;
}
