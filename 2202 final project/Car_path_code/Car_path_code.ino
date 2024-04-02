// sorter code





#define PRINT_COLOUR                                  // uncomment to turn on output of colour sensor data

#define LEFT_RAMP_SERVO         39  
#define RIGHT_RAMP_SERVO        40
#define FUNNEL_SERVO            41
#define SLIDE_SERVO             42

#define PLEFT_RAMP_SERVO         4
#define PRIGHT_RAMP_SERVO        5
#define PFUNNEL_SERVO            6
#define PSLIDE_SERVO             7


#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"
#include <MSE2202_Lib.h>

// Function declarations
void doHeartbeat();
long degreesToDutyCycle(int deg);

// Constants
const int cHeartbeatInterval = 75;                    // heartbeat update interval, in milliseconds
const int cSmartLED          = 21;                    // when DIP switch S1-4 is on, SMART LED is connected to GPIO21
const int cSmartLEDCount     = 1;                     // number of Smart LEDs in use
const int cSDA               = 47;                    // GPIO pin for I2C data
const int cSCL               = 48;                    // GPIO pin for I2C clock
const int cTCSLED            = 14;                    // GPIO pin for LED on TCS34725
const int cLEDSwitch         = 46;                    // DIP switch S1-2 controls LED on TCS32725    

// Constants for servos
const int leftRampServoUp = 150;               
const int leftRampServoDown = 75;

const int rightRampServoUp = 73;
const int rightRampServoDown = 148;

const int funnelServoStart = 171;
const int funnelServoMiddle = 108;
const int funnelServoEnd = 86;

const int slideServoGreen = 175;
const int slideServoOther = 90;

unsigned long MOVING_TIME = 3000; // moving time is 3 seconds

//timer
 int timekeeper = 0;
 int timer = 0;
 bool step1 = false;
 bool step2 = false;
 bool step3 = false;
 bool step4 = false;
 bool green = false;


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
 /* if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  } */


  //servo setup
  pinMode(LEFT_RAMP_SERVO, OUTPUT);                      // configure servo GPIO for output
   ledcSetup(PLEFT_RAMP_SERVO, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  ledcAttachPin(LEFT_RAMP_SERVO, PLEFT_RAMP_SERVO);         // assign servo pin to servo channel


  pinMode(RIGHT_RAMP_SERVO, OUTPUT);                      // configure servo GPIO for output
   ledcSetup(PRIGHT_RAMP_SERVO, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  ledcAttachPin(RIGHT_RAMP_SERVO, PRIGHT_RAMP_SERVO);         // assign servo pin to servo channel


  pinMode(FUNNEL_SERVO, OUTPUT);                      // configure servo GPIO for output
   ledcSetup(PFUNNEL_SERVO, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  ledcAttachPin(FUNNEL_SERVO, PFUNNEL_SERVO);         // assign servo pin to servo channel


  pinMode(SLIDE_SERVO, OUTPUT);                      // configure servo GPIO for output
   ledcSetup(PSLIDE_SERVO, 50, 14);                // setup for channel for 50 Hz, 14-bit resolution
  ledcAttachPin(SLIDE_SERVO, PSLIDE_SERVO);         // assign servo pin to servo channel
 
  
// inital servo pos
 ledcWrite(PLEFT_RAMP_SERVO,degreesToDutyCycle(leftRampServoDown)); 
 ledcWrite(PRIGHT_RAMP_SERVO,degreesToDutyCycle(rightRampServoDown));
 ledcWrite(PFUNNEL_SERVO,degreesToDutyCycle(funnelServoStart));
 ledcWrite(PSLIDE_SERVO,degreesToDutyCycle(slideServoGreen));


// for clock
prevMillis = millis();

}

void loop() {


curMillis = millis();

if ((curMillis-prevMillis)>5000){

  // angle finding
  unsigned long progress = millis() - 5000; // same as start time

  if (progress<MOVING_TIME){

   long angleL = map(progress, 0, MOVING_TIME, leftRampServoDown, leftRampServoUp);
   long angleR = map(progress, 0, MOVING_TIME, rightRampServoDown, rightRampServoUp);

   ledcWrite(PLEFT_RAMP_SERVO,degreesToDutyCycle(angleL)); 
   ledcWrite(PRIGHT_RAMP_SERVO,degreesToDutyCycle(angleR));
  }




   if (step1 == false) {
     timer = timer + 1;                          
      if (timer > 100000) {                             
                           // add timer 0 if you use if bead present below                            
        
       
       ledcWrite(PFUNNEL_SERVO,degreesToDutyCycle(funnelServoMiddle));

   step1 = true;
   timer = 0;
  }
 }
    


if (step2 == false && step1 == true) {

      uint16_t r, g, b, c;                                // RGBC values from TCS34725
 
 digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));    // turn on onboard LED if switch state is low (on position)
 //if (tcsFlag) {                                      // if colour sensor initialized
   tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
//#ifdef PRINT_COLOUR            
     Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
//#endif 
   //Check if the color is green (adjust the thresholds as needed)
   if (g > r && g > b && g > 40 && g < 60 && c < 100) {
    green = true;
    Serial.printf("sensed green\n");
    ledcWrite(PSLIDE_SERVO,degreesToDutyCycle(slideServoGreen));
  }
  else{
    green = false;
    Serial.printf("sensed other\n");
    ledcWrite(PSLIDE_SERVO,degreesToDutyCycle(slideServoOther));
  }
 //}


  timer = timer + 1;
      if (timer > 400){
        Serial.printf("%d\n",timer);
       ledcWrite(PFUNNEL_SERVO,degreesToDutyCycle(funnelServoEnd));  
       step2 = true;
       timer = 0;
      }

   }



     

   
     if (step3 == false && step2== true) {
     timer = timer + 1;
       if (timer > 100000){
        ledcWrite(PFUNNEL_SERVO,degreesToDutyCycle(funnelServoStart));
        timer = 0;
        step1 = false;
        step2 = false;
       
        
       }

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








// servo dutycycle
long degreesToDutyCycle(int deg) {
  const long cMinDutyCycle = 400;                     // duty cycle for 0 degrees
  const long cMaxDutyCycle = 2100;                    // duty cycle for 180 degrees

  long dutyCycle = map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);  // convert to duty cycle

#ifdef OUTPUT_ON // the were high peaks around 2 to 10 % in size of the total duty cycle. This must be the pwm controlling the position of the servo motor.
  float percent = dutyCycle * 0.0061039;              // (dutyCycle / 16383) * 100
  Serial.printf("Degrees %d, Duty Cycle Val: %ld = %f%%\n", servoPos, dutyCycle, percent);
#endif

  return dutyCycle;
}

