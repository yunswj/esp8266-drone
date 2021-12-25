#ifdef opticalFlow

int opticalUpdate = 15; //30
unsigned long opticalTs=millis();

 float optGain = 1;
 int limitOpt = 30;
 int16_t deltaY = 0, deltaX = 0 ,deltaX_= 0, deltaY_= 0, deltaCsr =0;
 int _flow_sum_x = 0;
 int _flow_sum_y = 0;

 float deltaCalY = 0;
 float deltaCalX = 0;


 
 int optTime = 0;

 int16_t Trim_Roll_Opt = 0;
 int16_t Trim_Pitch_Opt = 0;

 int16_t Trim_Roll_Opt_i = 0;
 int16_t Trim_Pitch_Opt_i = 0;

 int16_t Trim_Roll_Opt_i_M = 0;
 int16_t Trim_Pitch_Opt_i_M = 0;

 int opt_m_count =0;

 double attitudeOpt[3] = {0};

 float deltaCalXAnt = 0;
 float deltaCalYAnt = 0;

 float deltaCalXRate = 0;
 float deltaCalYRate = 0;

 String optSetupOutput(){

   String optOutput= "";

   optOutput = optOutput + "-----------------------------"  + '\n';
   optOutput = optOutput + "Optical Flow Calibraiton: " + '\n';
   optOutput = optOutput + "Roll_Opt: " ;
   optOutput = optOutput + Trim_Roll_Opt ;
   optOutput = optOutput + " Pitch_Opt: " ;
   optOutput = optOutput + Trim_Pitch_Opt + '\n' ;
   optOutput = optOutput + "Trim_Roll_Bs: " ;
   optOutput = optOutput + Trim_Roll_Bs ;
   optOutput = optOutput + " Trim_Pitch_Bs: " ;
   optOutput = optOutput + Trim_Pitch_Bs ;
   optOutput = optOutput + " Trim_Yaw_Bs: ";
   optOutput = optOutput + Trim_Yaw_Bs ;
   optOutput = optOutput + " done Trim_Bs" + '\n';
   

    return optOutput;
 }
 
 void opticalSensorSetup() {   
   Trim_Roll_Opt = word(EEPROM.read(35),EEPROM.read(36));
   Trim_Pitch_Opt = word(EEPROM.read(37),EEPROM.read(38));
   
   xOpt.SetItermRate(Trim_Roll_Opt);
   yOpt.SetItermRate(Trim_Pitch_Opt);

   Serial.print(" Trim_Roll_Opt: ");
   Serial.print(Trim_Roll_Opt);
   Serial.print(" Trim_Pitch_Opt: ");
   Serial.println(Trim_Pitch_Opt);
   
 }
void opticalSensor() {
  

 if((millis()-opticalTs)>opticalUpdate   ){  //Update only once per 10sec   && flyMode == 2 
 opticalTs = millis();

 Wire.requestFrom(8, 6);    // request 6 bytes from slave device #8
 int16_t response[6];
 int index = 0;
 // Wait for response
 while (Wire.available()) {
 int16_t b = Wire.read();
 response[index] = b;
 index++;
 }
  
  deltaY =word(response[0], response[1]);
  deltaX =word(response[2], response[3]);
  deltaCsr = word(response[4], response[5]);


  if( (deltaY + deltaX) == deltaCsr){
  // digitalWrite(2, 0);
  }else{
    deltaY =0;
    deltaX =0;
   //digitalWrite(2, 1);
  }
  if (isnan(deltaY)) {
    deltaY=0;
}

  if (isnan(deltaX)) {
    deltaX=0;
}


deltaX = constrain(deltaX, -limitOpt, limitOpt);
deltaY = constrain(deltaY, -limitOpt, limitOpt);


float alttidue = constrain(otoMeasure, 300, 700); // constrain(otoMeasure, 100, 1000); 

  float alttidueReverse = map(alttidue, 100, 1000, 1000, 100); 
  
  deltaCalX =  deltaX*(1500+alttidue);//1600+otoMeasure
  deltaCalY =  deltaY*(1500+alttidue); 


   if( deltaCalY >= 0){
   deltaCalYAnt = constrain((alttidueReverse/500)*(attitude[0]-attitudeOpt[0])*250,-deltaCalY,0 ); //  deltaCalYAnt = constrain((alttidueReverse/1000)*(attitude[0]-attitudeOpt[0])*250,-deltaCalY,0 );
   }else{
   deltaCalYAnt = constrain((alttidueReverse/500)*(attitude[0]-attitudeOpt[0])*250,0,-deltaCalY );
   }
   
   if( deltaCalX >= 0){
   deltaCalXAnt = constrain((alttidueReverse/500)*(attitude[1]-attitudeOpt[1])*250,-deltaCalX,0 );
   }else{
   deltaCalXAnt = constrain((alttidueReverse/500)*(attitude[1]-attitudeOpt[1])*250,0,-deltaCalX );
   }

   float trimPlusX;
   float trimPlusY;
   
   if(  yOpt.GetItermRateBase() > 0  ) {
   if( SetPointOpt[1] + RX_pitch_Multi > 0 ){
   trimPlusY = 1 + map(abs(yOpt.GetItermRateBase()),0,20000, 0, 1);
   }else{
   trimPlusY = 1 - map(abs(yOpt.GetItermRateBase()),0,20000, 0, 0.5);
   }
   }else{
   if( SetPointOpt[1] + RX_pitch_Multi > 0 ){
   trimPlusY = 1 - map(abs(yOpt.GetItermRateBase()),0,20000, 0, 0.5);
   }else{
   trimPlusY = 1 + map(abs(yOpt.GetItermRateBase()),0,20000, 0, 1);
   }
   }

   if( xOpt.GetItermRateBase()  > 0  ) {
   if( SetPointOpt[0] + RX_roll_Multi > 0 ){
   trimPlusX = 1 + map(abs(xOpt.GetItermRateBase()),0,20000, 0, 1);
   }else{
   trimPlusX = 1 - map(abs(xOpt.GetItermRateBase()),0,20000, 0, 0.5);
   }
   }else{
   if( SetPointOpt[0] + RX_roll_Multi > 0 ){
   trimPlusX = 1 - map(abs(xOpt.GetItermRateBase()),0,20000, 0, 0.5);
   }else{
   trimPlusX = 1 + map(abs(xOpt.GetItermRateBase()),0,20000, 0, 1);
   }
   }
   
   
   if(otoMeasure > 100  && autoOpt == true ){// 100
   xOpt.compute(4,(SetPointOpt[0] + RX_roll_Multi)*1000  * trimPlusX ,throttle,0,attitude[1],deltaCalX + deltaCalXAnt); //optGain*(deltaCalX - (alttidueReverse/1000)*rate[1]));
   yOpt.compute(5,(SetPointOpt[1] + RX_pitch_Multi)*1000 * trimPlusY ,throttle,0,attitude[0],deltaCalY + deltaCalYAnt); //optGain*(deltaCalY + (alttidueReverse/1000)*rate[0])) ;
   optTime = optTime +1;
  }else{ 
    xOpt.compute(4,0,throttle,0,attitude[1],0);
    yOpt.compute(5,0,throttle,0,attitude[0],0);
  }
  
  attitudeOpt[0] = attitude[0];
  attitudeOpt[1] = attitude[1];
  /*
    Serial.print( deltaY );
    Serial.print(" , ");
    Serial.print( deltaX );
    Serial.println();*/

  if(checkEepromOpt == 1 && armControl == 0 ){ // ??

    EEPROM.write(42,highByte(otoNewSet));
    EEPROM.write(43,lowByte(otoNewSet)); 

    EEPROM.write(35,highByte(Trim_Roll_Opt));
    EEPROM.write(36,lowByte(Trim_Roll_Opt));
    
    EEPROM.write(37,highByte(Trim_Pitch_Opt));
    EEPROM.write(38,lowByte(Trim_Pitch_Opt));
    EEPROM.commit();

    xOpt.SetItermRate(Trim_Roll_Opt);
    yOpt.SetItermRate(Trim_Pitch_Opt);
    checkEepromOpt = 0;
  }

  if( autoOpt == false ){
  optTime=0;
 }

   if( 4005 < opticalUpdate*optTime   &&  opticalUpdate*optTime < 4995){
    
    Trim_Roll_Opt_i_M = Trim_Roll_Opt_i_M +  xOpt.GetItermRate();
    Trim_Pitch_Opt_i_M = Trim_Pitch_Opt_i_M + yOpt.GetItermRate();
    opt_m_count = opt_m_count+1;
    
   }

  
  
   if( opticalUpdate*optTime ==  4995 ){ // after 5 seconds
  
  

   // Trim_Roll_Opt_i = Trim_Roll_Opt_i_M / opt_m_count;
   // Trim_Pitch_Opt_i = Trim_Pitch_Opt_i_M / opt_m_count;
    
    opt_m_count=0;
    Trim_Roll_Opt_i_M=0;
    Trim_Pitch_Opt_i_M=0;
    
    Trim_Roll_Opt_i = xOpt.GetItermRate();
    Trim_Pitch_Opt_i = yOpt.GetItermRate();
   
    if( Trim_Roll_Opt < Trim_Roll_Opt_i){
    Trim_Roll_Opt = Trim_Roll_Opt + round((Trim_Roll_Opt_i - Trim_Roll_Opt)/2); // (DistanceXint - DistanceX)/2
    }
    if( Trim_Roll_Opt > Trim_Roll_Opt_i){
    Trim_Roll_Opt = Trim_Roll_Opt - round((Trim_Roll_Opt - Trim_Roll_Opt_i)/2);
    }

    if( Trim_Pitch_Opt < Trim_Pitch_Opt_i){
    Trim_Pitch_Opt = Trim_Pitch_Opt + round((Trim_Pitch_Opt_i - Trim_Pitch_Opt)/2); // (DistanceXint - DistanceX)/2
    }
    if( Trim_Pitch_Opt > Trim_Pitch_Opt_i){
    Trim_Pitch_Opt = Trim_Pitch_Opt - round((Trim_Pitch_Opt - Trim_Pitch_Opt_i)/2);
    }
    
   checkEepromOpt = 1;
  }
   

}
}
#endif  
