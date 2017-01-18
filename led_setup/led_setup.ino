//Test program for LED strip control, I think we need seperate power for feeding them

#include <FastLED.h>


#define LED_PIN     A3
#define NUM_LEDS    16
#define BRIGHTNESS  100
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
int breath = 0;              //variable for red brightness
int bpmRed = 16;             //variable for breathing speed
byte ledNumber;
unsigned long timeSpend =0;
bool wait=false;
int gRunningOrder[] = {0,1,2,3};
// determine how many leds to light
int gNumLeds = sizeof(gRunningOrder)/sizeof(int);
void setup() {
  // put your setup code here, to run once:
   
    FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
    FastLED.setBrightness(  BRIGHTNESS ); 
    //standardLedSetup();
setAllYellow();
runningLight();
delay(3000);
     timeSpend=millis(); 
    
    
}

void loop() {
  // put your main code here, to run repeatedly:


 
runningLight();
   
}



void runningLight(){
  if(wait==false){
   if(ledNumber < gNumLeds-1) ledNumber++;
   else
    ledNumber=0;

   runningLightON(ledNumber);
   wait=true;
   timeSpend=millis();
}

if(millis()-timeSpend > 50) {
    wait=false;
    runnigLightOff();  
   }
}

void runningLightON(int ledNumber){
 
        // Turn our current led ON, then show the leds
        leds[gRunningOrder[ledNumber]] = CRGB::Red;
        leds[gRunningOrder[ledNumber]+4] = CRGB::Red;
        leds[gRunningOrder[ledNumber]+8] = CRGB::White;
        leds[gRunningOrder[ledNumber]+12] = CRGB::White;
        
 
        // Show the leds (only one of which is has a color set, from above
        // Show turns actually turns on the LEDs
        FastLED.show();
 
        // Wait a little bit - this will dictate speed  
}


void runnigLightOff(){

      // this will leave a tail
        fadeToBlackBy( leds, NUM_LEDS/3, 200);
        fadeToBlackBy( leds+4, NUM_LEDS/3, 200);
        fadeToBlackBy( leds+8, NUM_LEDS/3, 200);
        fadeToBlackBy( leds+12, NUM_LEDS/3, 200);
        //or un comment this to turn the leds off quickly, not leaving tail
        //leds[gRunningOrder[ledNumber]] = CRGB::Black;     
}



void turnOffLED(){
  
FastLED.clear();
fill_solid( leds, NUM_LEDS, CRGB(0,0,0));
FastLED.show(); 
}

void setAllYellow(){
 FastLED.clear();
fill_solid( leds, NUM_LEDS, CRGB(255,255,0));
FastLED.show();  
}

void setAllGreen(){
 FastLED.clear();
fill_solid( leds, NUM_LEDS, CRGB(0,255,0));
FastLED.show(); 
}

void standardLedSetup(){
  byte a;
  for (a=0;a<8;a++){
    leds[a] = CRGB::Red;
    leds[a+8] = CRGB::White; 
  }
  FastLED.show();
}

