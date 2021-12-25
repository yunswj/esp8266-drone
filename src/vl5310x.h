int otoMeasure;
int hoverlimitUp = 750;
int hoverlimitDown = 550;
int otoMeasure_;
int otoMeasure_0;
int changeMeasure;

float hoverFactor  = 0;
float hoverFactor_ = 450;
 
unsigned long otoPreviousMillis = 0;
const long otoInterval = 35;
int otoIntervalCont = 0;

int checkHandLand=0;

int takeBattery =0;

int checkEepromOto = 0;

int16_t yawItermRate = 0;
int takeOffTime = 32; //35
int takeOffTimeSecond = 70; 



void InitVL53L0X(){
  

  otoNewSet = word(EEPROM.read(42),EEPROM.read(43));
  
  yawItermRate = word(EEPROM.read(25),EEPROM.read(26));

  yaw.SetItermRate(yawItermRate);

  
  Serial.print(" yawItermRate: ");
  Serial.print(yawItermRate);
  Serial.print(" , ");

  
  Serial.print(" otoNewSet: ");
  Serial.println(otoNewSet);

  sensor.init();
  delay(10);
  sensor.setAddress((uint8_t)22);
  sensor.setTimeout(500);
  
  #if defined SHORT_RANGE
  sensor.startContinuous();
  #endif
  
  #if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif
}

void vl5310x(){
  
   unsigned long otoCurrentMillis = millis();
  if(otoCurrentMillis - otoPreviousMillis >= otoInterval){
  otoPreviousMillis = otoCurrentMillis;  
  
  //otoMeasure_ = sensor.readRangeSingleMillimeters(); 
   otoMeasure_ = sensor.readRangeContinuousMillimeters();

     
  if((otoMeasure_ < 2001)  && otoMeasure_ > 10  ){
  //float triOtoMeasure = 2*otoMeasure - (otoMeasure * cos(attitude_Radian[0])) - (otoMeasure * cos(attitude_Radian[1]))      ;
  //otoMeasure = otoMeasure - triOtoMeasure;
   otoMeasure = otoMeasure_;
   

   otoMeasure = (0.9*(float)otoMeasure) + ((1-0.9)*(float)otoMeasure_0); //0.65
   otoMeasure_0 = otoMeasure;

  
  if( otoHover == true && vl5310xControl == 1 ){
  if( takeBattery == 0){
  for(int i = 0;i<25;i++){
  oto.compute(3,targetOto,throttle,0,0,otoMeasure);
  hoverFactor_ = map(5.5*analogRead(A0),3600,4200,575,475) * weightOfDrone; //6*analogRead(A0)
  Serial.print("hoverFactor_= ");
  Serial.println(hoverFactor_);
  delay(1);
  }
  takeBattery =1; 
  }
  targetOto = constrain(targetOto,0,1000);
  oto.compute(3,targetOto,throttle,0,0,otoMeasure);
  hoverFactor =  constrain(hoverFactor_ +otoNewSet+ (oto.output/25),0,motorMax);  
  otoIntervalCont++;
  }
  
  if(otoIntervalCont == takeOffTime){ //35
  yawItermRate = yaw.GetItermRate(); 
  yaw.SetItermRate(yawItermRate);
  
  if((200-otoMeasure) < 0){
  otoNewSet= otoNewSet - map(abs(200-otoMeasure),0,200,0,50);///50
  }else{
  otoNewSet= otoNewSet + map(abs(200-otoMeasure),0,200,0,100);//50
  }   

  otoNewSet = constrain(otoNewSet,-75,75);
  checkEepromOto =1;
  }

   if(otoIntervalCont == takeOffTimeSecond){ //35  
     if(otoMeasure < 250){
      landingOff=0;
    }
  }
  
  

   if( otoIntervalCont > 100){
    if(otoMeasure <200){
     checkHandLand++;
    }else{
      checkHandLand=0;
    }
    if(checkHandLand == 3){
      armControl =0;
    }
   }

  
  
    if(checkEepromOto == 1 && throttle < 100 ){
    EEPROM.write(42,highByte(otoNewSet));
    EEPROM.write(43,lowByte(otoNewSet)); 

    EEPROM.write(25,highByte(yawItermRate));
    EEPROM.write(26,lowByte(yawItermRate)); 
    
    EEPROM.commit();
    delay(10);
    checkEepromOto = 0;
  }
   
  } else if( 2000 < otoMeasure_){
  otoMeasure= round(targetOto*1.3);
  
  }else if(0 > otoMeasure_ ){
  otoMeasure= 0;
  }
}
 
 if(otoHover == true  && vl5310xControl == 1){
 if( throttle < round(hoverFactor)){
 throttle = throttle +3;
 }
 if( throttle > round(hoverFactor)){
 throttle = throttle -3;
 }
 
 
 }else{
   otoIntervalCont =0;
    oto.SetInit(0,0,0);
    hoverFactor=0;
    takeBattery =0;
    hoverFactor_ =0;
  }

}

float getMeasureAltitude(){
 return otoMeasure;
}


String vlSetupOutput(){
   vl5310x();
   String vlOutput= "";
   vlOutput = vlOutput + "-----------------------------"  + '\n';
   vlOutput = vlOutput + "Altitute Calibration:"  + '\n'; 
   vlOutput = vlOutput + "Auto Set Value: " ; 
   vlOutput = vlOutput + otoNewSet  + '\n'; 
   vlOutput = vlOutput + "-----------------------------"  + '\n';
   vlOutput = vlOutput + "Yaw Calibration: " ; 
   vlOutput = vlOutput + yawItermRate +  '\n';
   vlOutput = vlOutput + "-----------------------------"  + '\n';
   vlOutput = vlOutput + "Altitute: " ; 
   vlOutput = vlOutput + getMeasureAltitude() +  '\n';
   return vlOutput; 
}
