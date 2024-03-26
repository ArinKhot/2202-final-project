#define PRINT_COLOUR                                  // uncomment to turn on output of colour sensor data

#define LEFT_RAMP_SERVO         39  
#define RIGHT_RAMP_SERVO        40
#define FUNNEL_SERVO            41
#define SLIDE_SERVO             42

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"
#include <MSE2202_Lib.h>

// Function declarations
void doHeartbeat();

// Constants
const int cHeartbeatInterval = 75;                    // heartbeat update interval, in milliseconds
const int cSmartLED          = 21;                    // when DIP switch S1-4 is on, SMART LED is connected to GPIO21
const int cSmartLEDCount     = 1;                     // number of Smart LEDs in use
const int cSDA               = 47;                    // GPIO pin for I2C data
const int cSCL               = 48;                    // GPIO pin for I2C clock
const int cTCSLED            = 14;                    // GPIO pin for LED on TCS34725
const int cLEDSwitch         = 46;                    // DIP switch S1-2 controls LED on TCS32725    

// Constants for servos
const int leftRampServoUp = 2200;                     // all temp values
const int leftRampServoDown = 1000;

const int rightRampServoUp = 2200;
const int rightRampServoDown = 1000;

const int funnelServoStart = 1000;
const int funnelServoMiddle = 1500;
const int funnelServoEnd = 2200;

const int slideServoGreen = 2200;
const int slideServoOther = 1000;

//timer
 int timer = 0;
 bool step1 = false;
 bool step2 = false;
 bool step3 = false;
 bool step4 = false;


// Variables
boolean heartbeatState       = true;                  // state of heartbeat LED
unsigned long lastHeartbeat  = 0;                     // time of last heartbeat state change
unsigned long curMillis      = 0;                     // current time, in milliseconds
unsigned long prevMillis     = 0;                     // start time for delay cycle, in milliseconds

// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number 
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);

// Smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 
                                       150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0};

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                     // TCS34725 flag: 1 = connected; 0 = not found

void setup() {
  Serial.begin(115200);                               // Standard baud rate for ESP32 serial monitor

  // Set up SmartLED
  SmartLEDs.begin();                                  // initialize smart LEDs object
  SmartLEDs.clear();                                  // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0,0,0)); // set pixel colours to black (off)
  SmartLEDs.setBrightness(0);                         // set brightness [0-255]
  SmartLEDs.show();                                   // update LED

  Wire.setPins(cSDA, cSCL);                           // set I2C pins for TCS34725
  pinMode(cTCSLED, OUTPUT);                           // configure GPIO to control LED on TCS34725
  pinMode(cLEDSwitch, INPUT_PULLUP);                  // configure GPIO to set state of TCS34725 LED 

  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }


  //servo setup

  Bot.servoBegin("S1", LEFT_RAMP_SERVO );  
  Bot.servoBegin("S2", RIGHT_RAMP_SERVO ); 
  Bot.servoBegin("S3", FUNNEL_SERVO );                                         
  Bot.servoBegin("S4", SLIDE_SERVO ); 

  Bot.ToPosition("S1",leftRampServoDown);
  Bot.ToPosition("S2",rightRampServoDown);
  Bot.ToPosition("S2",funnelServoStart);
  Bot.ToPosition("S2",slideServoGreen);

}

void loop() {




//servo demo

if (step1 == false) {
     
     timer = timer + 1;                          
     if (timer > 120000) {                             
        timer = 0;                                                
        step1 = true;
       // servo 1 pos change
       Bot.ToPosition("S1",leftRampServoUp);
     }
    }
    if (step2 == false && step1 == true) {
   
     timer = timer + 1;                                  
     if (timer > 120000) {                                        
        timer = 0;                                               
        step2 = true;
       // servo 2 pos change
       //servo 1 back 
       Bot.ToPosition("S1",leftRampServoDown);
       Bot.ToPosition("S2",rightRampServoUp);

     }
    }
  if (step3 == false && step2 == true) {
      timer = timer + 1;                            
     if (timer > 120000 ) {                                          
        timer = 0;                                             
        step3 = true;
       // servo 2 back 
       // servo 3 pos change
       Bot.ToPosition("S2",rightRampServoDown);
       Bot.ToPosition("S3",funnelServoMiddle);

     }
  }
  if (step4 == false && step3 == true) {
   
     timer = timer + 1;                                  
     if (timer > 120000) {                                          
        timer = 0;                                        
        step4 = true;
        // servo 3 end
        // servo 4 pos change 
        Bot.ToPosition("S3",funnelServoEnd);
        Bot.ToPosition("S4",slideServoOther);
     }
  }

  if (step4 == true) {
     timerCount3sec = timerCount3sec + 1;  
     if (timerCount2sec > 120000) {                                           
        timerCount3sec = 0;                                   

        step1 = false;
        step2 = false;
        step3 = false;
        step4 = false;                                                        // reset cycle

        // servo 4 back
        //servo 3 back
        Bot.ToPosition("S3",funnelServoStart);
        Bot.ToPosition("S4",slideServoGreen);
     }
  }



curMillis = millis();   
if (curMillis>=120){



  uint16_t r, g, b, c;                                // RGBC values from TCS34725
  
  digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));    // turn on onboard LED if switch state is low (on position)
  if (tcsFlag) {                                      // if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
#ifdef PRINT_COLOUR            
      Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
#endif
  }





} 
  doHeartbeat();                                      // update heartbeat LED
}













// update heartbeat LED
void doHeartbeat() {
  curMillis = millis();                               // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                        // update the heartbeat time for the next update
    LEDBrightnessIndex++;                             // shift to the next brightness level
    if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) { // if all defined levels have been used
      LEDBrightnessIndex = 0;                         // reset to starting brightness
    }
    SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]); // set brightness of heartbeat LED
    SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 250, 0)); // set pixel colours to green
    SmartLEDs.show();                                 // update LED
  }
}
