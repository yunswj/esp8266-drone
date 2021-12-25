#ifdef REMOTE_XY_REMOTE

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__ESP8266WIFI_LIB_POINT
#include <ESP8266WiFi.h>

#include <RemoteXY.h>

// RemoteXY connection settings 
#define REMOTEXY_WIFI_SSID "RemoteXY"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377


// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 255,6,0,51,0,59,0,8,24,0,
  5,43,53,7,41,41,31,26,24,5,
  0,1,8,43,43,31,26,24,2,1,
  37,2,23,9,24,26,31,31,65,82,
  77,0,68,73,83,65,82,77,0,3,
  6,45,12,7,38,31,26,67,5,8,
  55,86,5,135,26,51 };
  
// this structure defines all the variables of your control interface 
struct {

    // input variable
  int8_t joystick_1_x; // =-100..100 x-coordinate joystick position 
  int8_t joystick_1_y; // =-100..100 y-coordinate joystick position 
  int8_t joystick_2_x; // =-100..100 x-coordinate joystick position 
  int8_t joystick_2_y; // =-100..100 y-coordinate joystick position 
  uint8_t switch_1; // =1 if switch ON and =0 if OFF 
  uint8_t select_1; // =0 if select position A, =1 if position B, =2 if position C, ... 

    // output variable
  char text_1[51];  // string UTF8 end zero 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////


int checkTrottle=0;
int armControlCheck =0;
int armControlCheck_ =0;

int yawRange = 15;

void startRemotexy(){
   RemoteXY_Init();   
}

void receiveRMT(){
    RemoteXY_Handler(); 
    
    armControlCheck = RemoteXY.switch_1; // arm - disarm 
    if(armControlCheck==1){
    controlMethod=1;
    }
    if( armControlCheck_ != armControlCheck){
      armControl = armControlCheck;
    }
     armControlCheck_=armControlCheck;

    if(RemoteXY.joystick_2_y < -95){
     checkTrottle = 1;
    }
 
    if(checkTrottle == 1 || otoHover == 1){
    RX_throttle = (motorMax/100)*map(constrain(RemoteXY.joystick_2_y,-90,100),-90,100,0,100);
    }

    if(otoHover == 1   && landingOff == 1 && (checkEepromOpt == 1 ) ){//|| multiRangerDone == 1)
    targetOto =  map(RX_throttle,0,motorMax,100,1000);
    }
    
 
    int remoteXYSelect = RemoteXY.select_1;

   if(remoteXYSelect == 0){ // close 
     flyMode_1 = 0; //yön
     flyMode_2 = 0;//yükseklik
     flyMode_3 = 0; // optik
   }else if(remoteXYSelect == 1){
     flyMode_1 = 1; //yön
     flyMode_2 = 0;//yükseklik
     flyMode_3 = 0; // optik
   }else if(remoteXYSelect == 2){
     flyMode_1 = 0; //yön
     flyMode_2 = 1;//yükseklik
     flyMode_3 = 0; // optik
   }else if(remoteXYSelect == 3){
     flyMode_1 = 1; //yön
     flyMode_2 = 1;//yükseklik
     flyMode_3 = 0; // optik
   }else if(remoteXYSelect == 4){
     flyMode_1 = 0; //yön
     flyMode_2 = 1;//yükseklik
     flyMode_3 = 1; // optik
   }else if(remoteXYSelect == 5){
     flyMode_1 = 1; //yön
     flyMode_2 = 1;//yükseklik
     flyMode_3 = 1; // optik
   }

    RX_roll = RemoteXY.joystick_1_x;

    RX_pitch = RemoteXY.joystick_1_y;

    if(  RemoteXY.joystick_2_x  <= 0){
    RX_yaw =  map( constrain(RemoteXY.joystick_2_x,-100,-yawRange ),-100,-yawRange,-100,0) ;  
    }else{
    RX_yaw =  map( constrain(RemoteXY.joystick_2_x,yawRange,100 ),yawRange,100,0,100) ;
    }

}

#endif 

#ifdef FREECONTROL
void setupWiFi(){}   
void getRX(){}
#endif 

#ifdef REMOTE_XY_OWN

void startRemotexy(){
   RemoteXY_Init();   
}

void receiveRMT(){
  RemoteXY_Handler(); 
}
 
#endif 
