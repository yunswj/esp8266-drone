#include "pid.h"
#include "Parameter.h"



void PID::SetGain(float kp, float ki, float kpo, float kio,float kdo ){
  KP = kp;
  KI = ki;
  KPO = kpo;
  KIO=kio;
  KDO=kdo;
}
void PID::Setp(int32_t output_){
P = output_;
}
void PID::SetKI(float ki){
KI = ki;
}
void PID::SetKDO(float kd){
KDO = kd;
}

void PID::SetControl(int cn){
control = cn;
}

void PID::SetInit(int32_t p, int32_t d, int32_t i){
P=p;
I=i;
D=d;
}
void PID::SetLimit(int16_t error_limit, int16_t i_limit, int16_t output_limit){
  ERROR_LIMIT = error_limit;
  I_LIMIT = i_limit;
  OUTPUT_LIMIT = output_limit;
}
int16_t PID::GetItermRate(){
 return round(itermRate);
}
int16_t PID::GetItermRateBase(){
 return round(itermRateBase);
}
int PID::GetKPO(){
 return KPO;
}
void PID::SetItermRate(int32_t output_){
itermRateBase = output_;
}

void PID::compute(int whichIs, int32_t TARGET_ANGLE,int32_t THROTTLE,int32_t TRIM,int32_t MEASURED_ANGLE, int32_t MEASURED_RATE ){
  // Calculate delta
  timenow = millis();
  float dt = (float)(timenow - timeprev)/978.;
  timeprev = timenow;

   
  if(dt > -1){
  // Calculate Angle Error
  int32_t ANGLE_ERROR = (TARGET_ANGLE+TRIM ) - MEASURED_ANGLE;
 
  // Limit Error Signal
  ANGLE_ERROR = constrain(ANGLE_ERROR, -ERROR_LIMIT, ERROR_LIMIT);
  
  ITERM += ANGLE_ERROR * (float)dt;
  
  // Calculate I-term
   ITERM = constrain(ITERM, -I_LIMIT, I_LIMIT);

  
   if (THROTTLE <100 ) {
   ITERM  = 0;
   }
      
     // Get Desired Rate
  int32_t TARGET_RATE = ANGLE_ERROR * KP + KI * ITERM; // Calculate P-term

  if(whichIs == 2 &&  TARGET_ANGLE != 0){//whichIs == 2 || whichIs == 3 
    TARGET_RATE = TARGET_ANGLE;
  }
  

  if(whichIs == 5 || whichIs == 4 || whichIs == 3 || whichIs == 6 || whichIs == 7 ||whichIs == 8 ||whichIs == 9 ){//whichIs == 2 || whichIs == 3 
    TARGET_RATE = TARGET_ANGLE;
  }

  // Calculate Rate Error
  int32_t RATE_ERROR = TARGET_RATE - MEASURED_RATE;

  if(whichIs == 2 ){
   RATE_ERROR = constrain(RATE_ERROR, -2000, 2000);
  }
  
  if(whichIs == 3 ){
   RATE_ERROR = constrain(RATE_ERROR, -250, 250);
  }
  // if( whichIs == 4 || whichIs == 5 ){
  // RATE_ERROR = constrain(RATE_ERROR, 2000, 2000);
 // }
 
   if( whichIs == 6 || whichIs == 7 ){
   RATE_ERROR = constrain(RATE_ERROR, -125, 125);
  }
  
  itermRate= constrain(itermRate, -20000, 20000);

  
  // Calculate D-Term
  P = RATE_ERROR * KPO;

  delta = ((MEASURED_RATE - _MEASURED_RATE)/dt);
 
  _MEASURED_RATE= MEASURED_RATE;

  delta = 0.6 * (float)delta + (1. - 0.6) * (float)_delta;

  _delta = delta;


  D = (KDO/100) * delta;

   itermRate  = itermRate + RATE_ERROR * (float)dt;

 
  
   if (THROTTLE <100 ){
   itermRate = itermRateBase;
   }   
   



    if(whichIs == 3 ){
    if(RATE_ERROR < 0){
    KDO = map(RATE_ERROR, -250,0,250,900 );
    }else{
    KDO = map(RATE_ERROR, 0 ,250,900,250);
    }
     
    if(RATE_ERROR < 0){
    KPO = map(RATE_ERROR, -250,0,24,12 )/10;
    }else{
    KPO = map(RATE_ERROR, 0 ,250,12,24 )/10;
    }  
   
    if(RATE_ERROR < 0){
    KIO = map(RATE_ERROR, -250,0,35,15 )/10;//25,15 )
    }else{
    KIO = map(RATE_ERROR, 0 ,250,15,35 )/10;
    }   
   }

  if(KIO_ != KIO){
  itermRate  = (itermRate*KIO_)/KIO;
  }
  KIO_ = KIO;
  
   I = itermRate * KIO;
   
  output= P  + I - D ; //output= P + I - D;
  
  output = 0.95 * (float)output + (1 - 0.95) * (float)_output;
  _output = output;
  
  output = constrain(output, -OUTPUT_LIMIT, OUTPUT_LIMIT);

  }
}
