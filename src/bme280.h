//#ifndef bme280

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C


void bmeSetup(){
   bme.begin();    
}

float getBmeTemp(){
  if(bme280Control == 1){
     return bme.readTemperature();
  }else{
    return 0;
  }
}

float getBmeTemperature(){
  if(bme280Control == 1){
  return bme.readTemperature();
    }else{
   return 0;
  }
}

float getBmePressure(){
if(bme280Control == 1){
  return bme.readPressure() / 100.0F;
    }else{
   return 0;
 }
}

float getBmeAltitude(){
  if(bme280Control == 1){
  return bme.readAltitude(SEALEVELPRESSURE_HPA);
  }else{
   return 0;
  }
}

float getBmeHumidity(){
  if(bme280Control == 1){
  return bme.readHumidity();
    }else{
   return 0;
  }
}

//#endif
