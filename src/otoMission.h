

#define missionTime 30000

#ifdef otoMission
unsigned long baslagincZamani = 0;  
/*

                               __
                             /    \
                           /        \
                          |          |           
                          |          |
                          |   ++++   |
                          |          |
                          |          |
                          |          |
                          |Trim_Pitch|
                          

  ___________________                        ___________________
 /                   |                      |                   \
/       ----         |                      |       ++++         \
\                    |                      |                    /
 \____Trim_Roll______|                      |____Trim_Roll______/
 

                           Trim_Pitch
                          |          |           
                          |          |
                          |          |
                          |   ----   |
                          |          |
                          |          |
                          |          |
                           \        /
                             \ __ /


                      
*/

unsigned long kaydedilenZaman = 0;  

int saymayaBasla = 1;
int stopMission = 1;

int gorevNo = 0;

int sure[] = {  // milisaniye
  5000,
  5000,
  5000,
  5000,
  0,
};

int rollDegeri[] = {  
  0,
  500,
  1000,
  500, 
  0,
};

int pitchDegeri[] = {  
  0,
  500,
  0, 
  0,
  0,
};

int yukseklikDegeri[] = {  
  750,
  750,
  750,
  750,
  750
};

void setup_(){

}

void loop_(){
  if(flyMode_2 == 1 && stopMission == 1){
  unsigned long suAnkiZaman = millis();

  if(saymayaBasla == 1){
  kaydedilenZaman = suAnkiZaman; 
  baslagincZamani = suAnkiZaman;
  saymayaBasla = 0;
  }


  if( gorevNo < 5){
    
  if (suAnkiZaman - kaydedilenZaman >= sure[gorevNo] ) {
      kaydedilenZaman = suAnkiZaman;

   // RX_roll =  rollDegeri[gorevNo];
   // RX_pitch = pitchDegeri[gorevNo];
   //targetMultiX =  rollDegeri[gorevNo];
  // targetMultiY = pitchDegeri[gorevNo];
  
   targetOto = yukseklikDegeri[gorevNo];
   targetGOTX =  rollDegeri[gorevNo];
   targetGOTY = pitchDegeri[gorevNo];
   
    gorevNo = gorevNo + 1 ;
    }
  }else{
    gorevNo=0;
  }

  if (suAnkiZaman - baslagincZamani >= missionTime ) {
    
   // stopFlightControl = 0;
  //  stopMission = 0;
  }
  
}}
#endif  
