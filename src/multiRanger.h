

#ifdef MULTI_RANGER

VL53L0X sensor1, sensor2,sensor3,sensor4;

float DistanceX_, DistanceY_;

int targetMultiX1=300;
int targetMultiY1=300;

int targetMultiX0=300;
int targetMultiY0=300;

int multiRange= 1200;

int multiAlpha = 5;

int multiAlphaFree = 20;

#define TCAADDR 0x70

int pushButton = 2;
int multiRangerCounter=0;



void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

unsigned long multiPreviousMillis = 0;
const long multiInterval = 35;


void multiRangerSetup(){
  
   multiRange= 1200;
   targetMultiX1=1200;
   targetMultiY1=1200;

   targetMultiX0=1200;
   targetMultiY0=1200;
   
   #ifdef HandControl
   multiRange = 600;
   multiAlpha = 5;
   targetOto = 1000;
   DistanceX1 = 300; 
   DistanceY1 = 300;
   DistanceX0 = 300;
   DistanceY0 = 300;
  
   targetMultiX1=300;
   targetMultiY1=300;

   targetMultiX0=300;
   targetMultiY0=300;
      
   #endif


  #ifdef  AntiCollision

   multiRange = 600;
   multiAlpha = 5;
   targetOto = 1000;
   DistanceX1 = 600; 
   DistanceY1 = 600;
   DistanceX0 = 600;
   DistanceY0 = 600;
  
   targetMultiX1=600;
   targetMultiY1=600;

   targetMultiX0=600;
   targetMultiY0=600;
   
  
  #endif
   

  tcaselect(1);
  sensor1.init();
  sensor1.setTimeout(500);
  
  tcaselect(2);
  sensor2.init();
  sensor2.setTimeout(500);

  tcaselect(3);
  sensor3.init();
  sensor3.setTimeout(500);

  tcaselect(4);
  sensor4.init();
  sensor4.setTimeout(500);

  Wire.beginTransmission(TCAADDR);
  Wire.write(0);
  Wire.endTransmission();


  pinMode(pushButton, INPUT);
  
  
}

int valX1[5]; 
int valX0[5]; 
int valY1[5]; 
int valY0[5]; 

int valX1Med= 0;
int valY1Med= 0;

int valX0Med= 0;
int valY0Med= 0;

int dataValNo = 0;

int medianFilter =2;

int val3Int=0;
int val4Int=0;

int buttonStateC = 1;

void bubbleSort(int aray[(5)], int whichOne ) {
    boolean swapped = true;
    int j = 0;
    int tmp;
    while (swapped) {
        swapped = false;
        j++;
        for (int i = 0; i < 5 - j; i++) {
            if (aray[i] > aray[i + 1]) {
                tmp = aray[i];
                aray[i] = aray[i + 1];
                aray[i + 1] = tmp;
                swapped = true;
            }
        }
        if( whichOne == 0){
        for (int i = 0; i < (5) ; i++) {
        valY1[i] = aray[i];
        }}else if(whichOne == 1){
        for (int i = 0; i < (5) ; i++) {
        valX1[i] = aray[i];
        }}else if(whichOne == 2){
        for (int i = 0; i < (5) ; i++) {
        valX0[i] = aray[i];
        }}else if(whichOne == 3){
        for (int i = 0; i < (5) ; i++) {
        valY0[i] = aray[i];
        }}   
    }
}

int calSensor(int input, int target , int output){
 /// valY1Med
 if((input < multiRange)  &&  (input > 10) ){ // if((val1 < 1200)  &&  (val1 > 10) ){ //  if((valX1Med < 500)  &&  (valX1Med > 10) ){
 // DistanceY = valY1Med;
 int Distanceint = constrain(input,0,multiRange);
 if( output < Distanceint){
 output = output +round((Distanceint - output)/multiAlpha);
 }
 if( output > Distanceint){
 output = output - round((output - Distanceint)/multiAlpha);
 }  
  }else{
  //DistanceY= targetMultiX*1.5;
  //DistanceY= targetMultiY;
  //#ifdef HandControl
  if( output < target){
  output = output + round((target - output)/multiAlphaFree); // (DistanceXint - DistanceX)/2
  }
  if( output > target){
  output = output - round((output - target)/multiAlphaFree);
  }
  
  //#else 
 // output = -1;
 // #endif
  }
  return output;
  }

void multiRangerLoop(){

 
 unsigned long multiCurrentMillis = millis();
  if(multiCurrentMillis - multiPreviousMillis >= multiInterval){

   multiRangerCounter++;


    
  multiPreviousMillis = multiCurrentMillis;  

  tcaselect(1); 
  int val1Int = sensor1.readRangeSingleMillimeters();

  tcaselect(2); 
  int val2Int = sensor2.readRangeSingleMillimeters();

  tcaselect(3); 
  int val3Int = sensor3.readRangeSingleMillimeters();

  tcaselect(4); 
  int val4Int = sensor4.readRangeSingleMillimeters();
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(0);
  Wire.endTransmission();

   
  if(dataValNo < 5){
  valY1[dataValNo] = val1Int;
  valX1[dataValNo] = val2Int;
  valX0[dataValNo] = val4Int;
  valY0[dataValNo] = val3Int;
  dataValNo++;
  }
  
  if(dataValNo == 5){
  bubbleSort(valY1, 0);
  bubbleSort(valX1, 1);
  bubbleSort(valX0, 2);
  bubbleSort(valY0, 3);
  dataValNo=0;
  valY1Med = valY1[2];
  valX1Med = valX1[2];
  valX0Med = valX0[2];
  valY0Med = valY0[2];
  }

    
    DistanceY1 = calSensor(valY1Med,targetMultiY1,DistanceY1);
    DistanceX1 = calSensor(valX1Med,targetMultiX1,DistanceX1);

    DistanceY0 = calSensor(valY0Med,targetMultiY0,DistanceY0);
    DistanceX0 = calSensor(valX0Med,targetMultiX0,DistanceX0);


    Serial.print(" DistanceY1 ");
    Serial.print(DistanceY1);
    Serial.print(" DistanceX1 ");
    Serial.print(DistanceX1);
    Serial.print(" DistanceY0 ");
    Serial.print(DistanceY0);
    Serial.print(" DistanceX0 ");
    Serial.print(DistanceX0);
    Serial.println();




 #ifdef HandControl
if(otoMeasure > 100){
  xMulti.compute(6,targetMultiX1,throttle,0,0,DistanceX1);
  yMulti.compute(7,targetMultiY1,throttle,0,0,DistanceY1);
  
  xMultiM.compute(6,targetMultiX0,throttle,0,0,DistanceX0);
  yMultiM.compute(7,targetMultiY0,throttle,0,0,DistanceY0);
  
}else{
  xMulti.compute(6,targetMultiX1,throttle,0,0,targetMultiX1);
  yMulti.compute(7,targetMultiY1,throttle,0,0,targetMultiY1);

  xMultiM.compute(6,targetMultiX0,throttle,0,0,targetMultiX0);
  yMultiM.compute(7,targetMultiY0,throttle,0,0,targetMultiY0);
}

   // Serial.print("HANDDDDD: ");


#endif

 #ifdef AntiCollision

   int antiX  = map(DistanceX1, 0, 600, 0, -15);
   int antiX_  = map(DistanceX0, 0, 600, 0, +15);

    RX_roll_Multi = antiX_ + antiX;

   int antiY  = map(DistanceY1, 0, 600, 0, -15);
   int antiY_  = map(DistanceY0, 0, 600, 0, 15);

    RX_pitch_Multi = antiY_ + antiY;

 #endif
  
  }
}
  

#endif  
