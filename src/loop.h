
unsigned long previousMillisabc = 0;      
void mainLoop(){
  //unsigned long getRXTime=micros();
  getRX();   
  modeControl();
  FlightControl();  
 // unsigned long getRXTime2=micros();

    /*
 // if( getRXTime2 - getRXTime > 500){
   Serial.print("  getRX: ");
   Serial.print( getRXTime2 - getRXTime );
   Serial.println();
   */
}
