#ifdef NeoPixel
#include <NeoPixelBus.h>

const uint16_t PixelCount = 12; // this example assumes 4 pixels, making it smaller will cause a failure
const uint8_t PixelPin = 2;  // make sure to set this to the correct pin, ignored for Esp8266

#define colorSaturation 128

NeoPixelBus<NeoGrbFeature, NeoEsp8266Uart1800KbpsMethod> strip(PixelCount, PixelPin);
//NeoPixelBus<NeoRgbFeature, NeoEsp8266Uart1400KbpsMethod> strip(PixelCount, PixelPin);



unsigned long pixelsInterval=50;  // the time we need to wait
unsigned long colorWipePreviousMillis=0;
unsigned long theaterChasePreviousMillis=0;
unsigned long theaterChaseRainbowPreviousMillis=0;
unsigned long rainbowPreviousMillis=0;
unsigned long rainbowCyclesPreviousMillis=0;

int theaterChaseQ = 0;
int theaterChaseRainbowQ = 0;
int theaterChaseRainbowCycles = 0;
int rainbowCycles = 0;
int rainbowCycleCycles = 0;

uint16_t currentPixel = 0;// what pixel are we operating on

int startNeo = 0;
int neoCheck=0;
   //RgbColor updatedColor = RgbColor::LinearBlend( WheelPos * 3, 255 - WheelPos * 3,0 );
   
RgbColor  Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return RgbColor(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return RgbColor(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return RgbColor(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void rainbow() {
  for(uint16_t i=0; i<PixelCount; i++) {
    strip.SetPixelColor(i, Wheel((i+rainbowCycles) & 255));
  }
  strip.Show();
  rainbowCycles++;
  if(rainbowCycles >= 256) rainbowCycles = 0;
}


void NeoPixelsetup() {
 
 currentPixel = 0;
 
 strip.Begin();
 strip.Show();
      
}

void neoPixeCheck(){
    if( neoCheck == 0){
    NeoPixelsetup();
    for(int i =0; i <12; i++){
    RgbColor newColor = RgbColor(0,0,0);
    strip.SetPixelColor(i, newColor);
    }
    strip.Show();
    neoCheck=1;
}
}

void ESPrainbow(){
   neoPixeCheck();
   if ((unsigned long)(millis() - rainbowPreviousMillis) >= pixelsInterval) {
       rainbowPreviousMillis = millis();
       rainbow();
    }
}

void ESPsetPixel(int n,int r ,int g, int b){
  neoPixeCheck();
 RgbColor newColor = RgbColor(r,g,b);
 strip.SetPixelColor(n, newColor);
}

void ESPpixelShow(){
strip.Show();
}


#endif
