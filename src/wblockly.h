#ifdef STANDALONE

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <FS.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

//#include <ESP8266mDNS.h>

String AP_NameString = "ESPCopter";
String AP_PassString = "123456789";

ESP8266WebServer server_1(80);       // create a web server on port 80
WebSocketsServer webSocket(81);    // create a websocket server on port 81

File fsUploadFile;             // a File variable to temporarily store the received file



int wifiMode = 1;


void update_wifi_mode(int modeSelection){
    EEPROM.write(100,modeSelection); 
    EEPROM.commit();
    if(modeSelection){
    Serial.println("Access point (You need to connect ESPcopter)");
    }else{
    Serial.println("Station point (ESPcopter willconnect your device)");
    }
}

    

 void update_SSID_PASS(const char* ssidGt , const char* passGt ){


  if(strlen(passGt) < 8){
  passGt ="";  
  }
  
  SPIFFS.remove("/f.txt");
  
  File f = SPIFFS.open("/f.txt", "r");
  
  if (!f) {
    Serial.println("File doesn't exist yet. Creating it");
    delay(1000);
    // open the file in write mode
    File f = SPIFFS.open("/f.txt", "w");
    if (!f) {
      Serial.println("file creation failed");
    }
    // now write two lines in key/value style with  end-of-line characters
    Serial.println("writting");
    f.println(ssidGt);
    f.println(passGt);
    f.close();
  }
     Serial.print("New SSID: " );
     Serial.print(ssidGt);
     Serial.print(" , , ");
     Serial.print("New PASS: " );
     Serial.print(passGt);
     Serial.println();
}


void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length){
  
     
   char payloadMsg[length];
   
   if (type == WStype_TEXT){
   for(int i = 0; i < length; i++) {
   Serial.print((char) payload[i]);
   payloadMsg[i] = payload[i];
   }
   Serial.println();
   }
   
    //StaticJsonDocument<512> root_MQTT;//1024//bug?20171016
    DynamicJsonDocument root_MQTT(1024);
    StaticJsonDocument<200> doc;
    
    DeserializationError error = deserializeJson(root_MQTT, payloadMsg);

      

  
  if (error == 0) {
    //blue green red: {"color":[0,0,0]} // {"cal":["0"]}


    
    
    if (root_MQTT.containsKey("color"))
    {
      blue=root_MQTT["color"][0];
      green=root_MQTT["color"][1];
      red=root_MQTT["color"][2];
    }

    if (root_MQTT.containsKey("arm")){

      int arm = root_MQTT["arm"];
      if(arm==0){
      armControl = 0;
      }
      if(arm==1){
      armControl = 1; 
      }
    }

    
    if (root_MQTT.containsKey("mode1"))
    {
      int mode1 = root_MQTT["mode1"];
      if(mode1==0){
      flyMode_1 =0;
      }
      if(mode1==1){
      flyMode_1 =1;
      }
    }

  if (root_MQTT.containsKey("mode2"))
    {
      int mode2 = root_MQTT["mode2"];
      if(mode2==0){
    flyMode_2 =0;
      }
      if(mode2==1){
    flyMode_2 =1;
      }
    }
    
    if (root_MQTT.containsKey("mode3"))
    {
      int mode3 = root_MQTT["mode3"];
      if(mode3==0){
       flyMode_3 =0;
      }
      if(mode3==1){
      flyMode_3 =1;
      }
    }

     if (root_MQTT.containsKey("land")){
      int landing = root_MQTT["land"];
      landingOff=0;
    }

      if (root_MQTT.containsKey("takeOff")){
      int takeOff = root_MQTT["takeOff"];
      armControl = 1;
      //flyMode_1 = 1;
      flyMode_3 = 1;
      
    }
     
    //throttle: {"throttle":0}

      if (root_MQTT.containsKey("setAltitude")){
      int newAltitude = root_MQTT["setAltitude"];
      targetOto=float(newAltitude);
    }
    
    if (root_MQTT.containsKey("throttle")){
      int throttle_temp = root_MQTT["throttle"];
      RX_throttle=float(throttle_temp);
    }
    
      if (root_MQTT.containsKey("yaw")){
      int yaw = root_MQTT["yaw"];
      RX_yaw=float(yaw);
    }
      if (root_MQTT.containsKey("roll")){
      int roll = root_MQTT["roll"];
      RX_roll=float(roll);
    }
      if (root_MQTT.containsKey("pitch")){
      int pitch = root_MQTT["pitch"];
      RX_pitch=float(pitch);
    }
    
    //trimX trimY trimZ:  {"trimXYZ":[100,100,100]}
    if (root_MQTT.containsKey("trimXYZ"))
    {
      Trim_Roll=root_MQTT["trimXYZ"][0];
      Trim_Pitch=root_MQTT["trimXYZ"][1];
      Trim_Yaw=root_MQTT["trimXYZ"][2];
    }

     if (root_MQTT.containsKey("rgbM")) {
      
     int ledNO =root_MQTT["rgbM"][0];
     int r =root_MQTT["rgbM"][1];
     int g =root_MQTT["rgbM"][2];
     int b =root_MQTT["rgbM"][3];
     ESPsetPixel(ledNO, r,g,b);
     ESPpixelShow();
    }
    
    
    //xAxis yAxis zAxis:  {"xyzAxis":[200,200,200]}
    if (root_MQTT.containsKey("xyzAxis"))
    {
      RX_roll=root_MQTT["xyzAxis"][0];
      RX_pitch=root_MQTT["xyzAxis"][1];
      RX_yaw=root_MQTT["xyzAxis"][2];
    }


    if (root_MQTT.containsKey("setLed"))
    {
     int ledNo =  Trim_Roll=root_MQTT["setLed"][0];
     int setting  = Trim_Pitch=root_MQTT["setLed"][1];
     if(ledNo == 0){
      esp.redLed_Digital(setting);
     }else if(ledNo == 1){
       esp.greenLed_Digital(setting);
     }else{
      esp.blueLed_Digital(setting);
     }
    }

    if (root_MQTT.containsKey("setMotor")){
     int motorNo =  Trim_Roll=root_MQTT["setMotor"][0];
     int setting  = Trim_Pitch=root_MQTT["setMotor"][1];

     
     if(motorNo == 0){
     setMotorSpeedFL(constrain(setting,0,PWM_PERIOD));
     }else if(motorNo == 1){
     setMotorSpeedFR(constrain(setting,0,PWM_PERIOD));
     }else if(motorNo == 2){
     setMotorSpeedRR(constrain(setting,0,PWM_PERIOD));
     }else{
     setMotorSpeedRL(constrain(setting,0,PWM_PERIOD));
     }

    }

     if (root_MQTT.containsKey("buzzer")){
      int data = root_MQTT["buzzer"]; 
      if(data==0){
      buzzerControl = 0;
      }
      if(data==1){
      buzzerControl = 1; 
      }
    }
    
    
    
//----------------------------------------------------------------

    if (root_MQTT.containsKey("goForward"))
    {
    int timing = root_MQTT["goForward"];
    blockly.goForward(timing);
    }
    if (root_MQTT.containsKey("goBack")){
    int timing = root_MQTT["goBack"];
    blockly.goBack(timing);
    }
    if (root_MQTT.containsKey("goLeft")){
    int timing = root_MQTT["goLeft"];
    blockly.goLeft(timing);
    }
    if (root_MQTT.containsKey("goRight")){
    int timing = root_MQTT["goRight"];
    blockly.goRight(timing);
    }

   if (root_MQTT.containsKey("turnRight"))
    {
    int value = root_MQTT["turnRight"];
    blockly.turnRight(value);
    }
    
      if (root_MQTT.containsKey("turnLeft"))
    {
    int value = root_MQTT["turnLeft"];
    blockly.turnLeft(value);
    }

//-----------------------------


    if (root_MQTT.containsKey("wifiMode")){
    if(root_MQTT["wifiMode"] == 1){
    update_wifi_mode(1);
    }
    else{
    update_wifi_mode(0);
    }
    }



  
  if (root_MQTT.containsKey("ssidPass")){

     const char* ssidGt =  root_MQTT["ssidPass"][0];
     const char* passGt  = root_MQTT["ssidPass"][1];
      
     update_SSID_PASS(ssidGt,passGt);
  
  
  }


   if (root_MQTT.containsKey("cal")){
     int cal = root_MQTT["cal"]; 
     Serial.print("passGt: " );
     Serial.println(cal);
     ahrs.calibraiton(cal);
     ahrs.resetAutoCalibraitons();
    }

    
    
  }
}

String mainOutput(){
String mainOutput= "";

    mainOutput = mainOutput + "-----------------------------"  + '\n';
    
    mainOutput = mainOutput + "Waiting Connecting..."  + '\n';
    mainOutput = mainOutput + "SSID: ";
    mainOutput = mainOutput + AP_NameString + '\n' ;
    mainOutput = mainOutput + "PASS: ";
    mainOutput = mainOutput + AP_PassString + '\n'; 
    mainOutput = mainOutput + "Wifi Mode: ";
    if(wifiMode){
    mainOutput = mainOutput + "Access point (You need to connect ESPcopter)" + '\n';
    }else{
    mainOutput = mainOutput + "Station point (ESPcopter willconnect your device)" + '\n';
    }
    mainOutput = mainOutput + "-----------------------------"  + '\n';
    mainOutput = mainOutput + "Battery Volt: ";
    mainOutput = mainOutput + getBatteryVoltage() + '\n';
    mainOutput = mainOutput + "Battery Level: %";
    mainOutput = mainOutput + getBatteryLevel() + '\n';
    mainOutput = mainOutput + "-----------------------------"  + '\n';
    mainOutput = mainOutput + "Founded Modules: " + '\n';

    if(vl5310xControl==1){
    mainOutput = mainOutput + "Altidude hold shield was found" + '\n';
    }else{
    mainOutput = mainOutput + "Altidude hold shield was not found" + '\n';
    }

    if(opticalFlowControl==1){
    mainOutput = mainOutput + "Optical hold shield was found " + '\n';
    }else{
    mainOutput = mainOutput + "Optical hold shield was not found" + '\n';
    }
    
    if(multiRangerControl==1){
    mainOutput = mainOutput + "Multi-ranger shield was found " + '\n';
    }else{
    mainOutput = mainOutput + "Multi-ranger shield was not found" + '\n';
    }

    if(bme280Control==1){
    mainOutput = mainOutput + "BME280 shield was found " + '\n';

    }else{
    mainOutput = mainOutput + "BME280 shield was not found"+ '\n';
    }
    
    if(lpsControl==1){
    mainOutput = mainOutput + "Custom shield was found "+ '\n';
    }else{
    mainOutput = mainOutput + "Custom shield was not found"+ '\n';
    }
  return mainOutput;
} 


String allOutput(){
 
 String allOutput= "";
 allOutput = allOutput +  mainOutput();
 allOutput = allOutput +  ahrs.setupOutput();
 allOutput = allOutput +  optSetupOutput();
 allOutput = allOutput +  vlSetupOutput();
 return allOutput;
}
void setupOutput(){
    Serial.print(allOutput());
    Serial.println(); 
    Serial.println(); 
}

String debugOutput(){
 return "test";  
}

void apMode(){
  
  WiFi.mode(WIFI_AP);

  // Do a little work to get a unique-ish name. Append the
  // last two bytes of the MAC (HEX'd) to "Thing-":
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();

  
  WiFi.softAP(AP_NameString, AP_PassString);

  while ( WiFi.softAPgetStationNum() < 1) {  // Wait for the Wi-Fi to connect 
  delay(250);
  setupOutput();
  int buttonState = digitalRead(2);
  if( buttonState == 0 && buttonStateC == 1){
  buttonStateC=0;
  armControl = 1;
  flyMode_2 = 1;
  flyMode_3 = 1;
  delay(2000);
  break;
  }
  }
}

void staMode(){

    WiFi.mode(WIFI_AP_STA);  
  
    WiFi.begin(AP_NameString, AP_PassString);


  while (WiFi.status() != WL_CONNECTED ) {
   delay(250);
  setupOutput();
  int buttonState = digitalRead(2);
  if( buttonState == 0 && buttonStateC == 1){
  buttonStateC=0;
  armControl = 1;
  flyMode_2 = 1;
  flyMode_3 = 1;
  delay(2000);
  Serial.print(".");
  }

}

  
}

void startWiFi() { // Start a Wi-Fi access point, and try to connect to some given access points. Then wait for either an AP or STA connection

  wifiMode = EEPROM.read(100);
  
  #ifdef STA_Mode
  wifiMode=0;
  #endif 

  #ifdef AP_Mode
  wifiMode=1;
  #endif 

  if(wifiMode ==0){
  staMode();
  }else if(wifiMode = 1){
  apMode();
  }else{
  apMode(); 
  }
/*
   if (!MDNS.begin("espcopter")) {
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  
   //MDNS.addService("http", "tcp", 80);
   Serial.println("mDNS responder started");
  */
  
  }


String formatBytes(size_t bytes) { // convert sizes in bytes to KB and MB
  if (bytes < 1024) {
    return String(bytes) + "B";
  } else if (bytes < (1024 * 1024)) {
    return String(bytes / 1024.0) + "KB";
  } else if (bytes < (1024 * 1024 * 1024)) {
    return String(bytes / 1024.0 / 1024.0) + "MB";
  }
}

void startSPIFFS() { // Start the SPIFFS and list all contents
  SPIFFS.begin();                             // Start the SPI Flash File System (SPIFFS)


  #ifdef ESPCOPTER_WIFI_SSID
  update_SSID_PASS(ESPCOPTER_WIFI_SSID , ESPCOPTER_WIFI_PASSWORD);
  #endif 


  
  Serial.println("SPIFFS started. Contents:");
  {
    Dir dir = SPIFFS.openDir("/");
    while (dir.next()) {                      // List the file system contents
      String fileName = dir.fileName();
      size_t fileSize = dir.fileSize();
      Serial.printf("\tFS File: %s, size: %s\r\n", fileName.c_str(), formatBytes(fileSize).c_str());
    }
    Serial.printf("\n");
  }


 File f = SPIFFS.open("/f.txt", "r");

 if (!f) {
    Serial.println("File doesn't exist yet. Creating it");

    // open the file in write mode
    File f = SPIFFS.open("/f.txt", "w");
    if (!f) {
      Serial.println("file creation failed");
    }
    // now write two lines in key/value style with  end-of-line characters
    f.println("ESPcopter");
    f.println("123456789");
  } else {
    
    // we could open the file


     AP_NameString = f.readStringUntil('\r');              
     AP_PassString = f.readStringUntil('\r');
            
    f.close();
  
    AP_NameString.replace("\n", "");
     AP_PassString.replace("\n", "");

   
     Serial.print("SSID:");
     Serial.println(AP_NameString);
     
     Serial.print("PASS:");
     Serial.println(AP_PassString);


    

  }
  
}

void startWebSocket() { // Start a WebSocket server
  webSocket.begin();                          // start the websocket server
  webSocket.onEvent(webSocketEvent);          // if there's an incomming websocket message, go to function 'webSocketEvent'
  Serial.println("WebSocket server started.");
}


void handleFileUpload(){ // upload a new file to the SPIFFS
  HTTPUpload& upload = server_1.upload();
  String path;
  if(upload.status == UPLOAD_FILE_START){
    path = upload.filename;
    if(!path.startsWith("/")) path = "/"+path;
    if(!path.endsWith(".gz")) {                          // The file server always prefers a compressed version of a file 
      String pathWithGz = path+".gz";                    // So if an uploaded file is not compressed, the existing compressed
      if(SPIFFS.exists(pathWithGz))                      // version of that file must be deleted (if it exists)
         SPIFFS.remove(pathWithGz);
    }
    Serial.print("handleFileUpload Name: "); Serial.println(path);
    fsUploadFile = SPIFFS.open(path, "w");            // Open the file for writing in SPIFFS (create if it doesn't exist)
    path = String();
  } else if(upload.status == UPLOAD_FILE_WRITE){
    if(fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize); // Write the received bytes to the file
  } else if(upload.status == UPLOAD_FILE_END){
    if(fsUploadFile) {                                    // If the file was successfully created
      fsUploadFile.close();                               // Close the file again
      Serial.print("handleFileUpload Size: "); Serial.println(upload.totalSize);
      server_1.sendHeader("Location","/success.html");      // Redirect the client to the success page
      server_1.send(303);
    } else {
      server_1.send(500, "text/plain", "500: couldn't create file");
    }
  }
}

String getContentType(String filename) { // determine the filetype of a given filename, based on the extension
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}


bool handleFileRead(String path) { // send the right file to the client (if it exists)

  if(path == "/main.css" ){
  controlMethod = 2;
  Serial.print("controlMethod: ");
  Serial.println(controlMethod);
  }
  
  Serial.println("handleFileRead: " + path);
  if (path.endsWith("/")) path += "index.html";          // If a folder is requested, send the index file
  String contentType = getContentType(path);             // Get the MIME type
  String pathWithGz = path + ".gz";
  if (SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)) { // If the file exists, either as a compressed archive, or normal
    if (SPIFFS.exists(pathWithGz))                         // If there's a compressed version available
      path += ".gz";                                         // Use the compressed verion
    File file = SPIFFS.open(path, "r");                    // Open the file
    size_t sent = server_1.streamFile(file, contentType);    // Send it to the client
    file.close();                                          // Close the file again
    Serial.println(String("\tSent file: ") + path);
    return true;
  }
  Serial.println(String("\tFile Not Found: ") + path);   // If the file doesn't exist, return false
  return false;
}

void handleNotFound(){ // if the requested file or page doesn't exist, return a 404 not found error
  if(!handleFileRead(server_1.uri())){          // check if the file exists in the flash memory (SPIFFS), if so, send it
    server_1.send(404, "text/plain", "404: File Not Found");
  }
}


void sendBattery() {
  server_1.send(200, "text/plane", String(getBatteryVoltage())); //Send ADC value only to client ajax request ??
}

void sendImuTemp() {
  server_1.send(200, "text/plane", String(getMpuTemp())); //Send ADC value only to client ajax request ??
}

void sendBme280T() {
  server_1.send(200, "text/plane", String(getBmeTemp())); //Send ADC value only to client ajax request ??
   Serial.println(String(getBmeTemp()));
}
void sendBme280P() {
  server_1.send(200, "text/plane", String(getBmePressure())); //Send ADC value only to client ajax request ??
}
void sendBme280H() {
  server_1.send(200, "text/plane", String(getBmeHumidity())); //Send ADC value only to client ajax request ??
}

void sendAltitude() {
  server_1.send(200, "text/plane", String(getOtoMeasure())); //Send ADC value only to client ajax request ??
}

void sendSfVersion() {
  server_1.send(200, "text/plane", getSfVersion()); //Send ADC value only to client ajax request ??
}

void sendSpVersion() {
  server_1.send(200, "text/plane", getSpVersion()); //Send ADC value only to client ajax request ??
}

void sendDebug() {
  server_1.send(200, "text/plane", allOutput()); //Send ADC value only to client ajax request ??
}

void sendMultiRangerX0() {
  server_1.send(200, "text/plane", String(Distance_X_0())); //Send ADC value only to client ajax request ??
}

void sendMultiRangerX1() {
  server_1.send(200, "text/plane", String(Distance_X_1())); //Send ADC value only to client ajax request ??
}

void sendMultiRangerY0() {
  server_1.send(200, "text/plane", String(Distance_Y_0())); //Send ADC value only to client ajax request ??
}

void sendMultiRangerY1() {
  server_1.send(200, "text/plane", String(Distance_Y_1())); //Send ADC value only to client ajax request ??
}


void sendOptX() {
  server_1.send(200, "text/plane", String(getOptData_X())); //Send ADC value only to client ajax request ??
}

void sendOptY() {
  server_1.send(200, "text/plane", String(getOptData_Y())); //Send ADC value only to client ajax request ??
}

void sendAccelX() {
  server_1.send(200, "text/plane", String(ahrs.getAccelX())); //Send ADC value only to client ajax request ??
}

void sendAccelY() {
  server_1.send(200, "text/plane", String(ahrs.getAccelY())); //Send ADC value only to client ajax request ??
}

void sendAccelZ() {
  server_1.send(200, "text/plane", String(ahrs.getAccelZ())); //Send ADC value only to client ajax request ??
}

void sendGyroX() {
  server_1.send(200, "text/plane", String(ahrs.getGyroX())); //Send ADC value only to client ajax request ??
}

void sendGyroY() {
  server_1.send(200, "text/plane", String(ahrs.getGyroY())); //Send ADC value only to client ajax request ??
}

void sendGyroZ() {
  server_1.send(200, "text/plane", String(ahrs.getGyroZ())); //Send ADC value only to client ajax request ??
}



void startServer() { // Start a HTTP server with a file read handler and an upload handler
  server_1.on("/edit.html",  HTTP_POST, []() {  // If a POST request is sent to the /edit.html address,
    server_1.send(200, "text/plain", ""); 
  }, handleFileUpload);                       // go to 'handleFileUpload'

  server_1.onNotFound(handleNotFound);        // if someone requests any other file or page, go to function 'handleNotFound'
                                                                                  
  server_1.on("/readBattery", sendBattery);              //??     
  server_1.on("/readImuTemp", sendImuTemp);  

  server_1.on("/readBme280T", sendBme280T);  
  server_1.on("/readBme280P", sendBme280P);  
  server_1.on("/readBme280H", sendBme280H);  

  server_1.on("/readAltitude", sendAltitude);  

  server_1.on("/readSfVersion", sendSfVersion);  
  server_1.on("/readSpVersion", sendSpVersion);  

  server_1.on("/readMultiRangerX0", sendMultiRangerX0);  
  server_1.on("/readMultiRangerX1", sendMultiRangerX1);  
  server_1.on("/readMultiRangerY0", sendMultiRangerY0);  
  server_1.on("/readMultiRangerY1", sendMultiRangerY1);  

  server_1.on("/readOptX", sendOptX);  
  server_1.on("/readOptY", sendOptY);  

  server_1.on("/readAccelX", sendAccelX);  
  server_1.on("/readAccelY", sendAccelY);  
  server_1.on("/readAccelZ", sendAccelZ);  

  server_1.on("/readGyroX", sendGyroX);  
  server_1.on("/readGyroY", sendGyroY);  
  server_1.on("/readGyroZ", sendGyroZ);  


  server_1.on("/debug", sendDebug);  
                    
  server_1.begin();                           // start the HTTP server
  Serial.println("HTTP server started.");
}



void getRX(){
  if(controlMethod == 0){
  receiveRMT(); 
  webSocket.loop();                             // constantly check for websocket events
  server_1.handleClient();                      // run the server
  }else if( controlMethod== 1){
  receiveRMT(); 
  }else if( controlMethod== 2){
  webSocket.loop();                             // constantly check for websocket events
  server_1.handleClient();                      // run the server
 }
}




void setupWiFi(){  

  startSPIFFS();               // Start the SPIFFS and list all contents

  startWiFi();                 // Start a Wi-Fi access point, and try to connect to some given access points. Then wait for either an AP or STA connection
  
  startWebSocket();            // Start a WebSocket server

  startServer();               // Start a HTTP server with a file read handler and an upload handler

  startRemotexy();

}


#endif  
