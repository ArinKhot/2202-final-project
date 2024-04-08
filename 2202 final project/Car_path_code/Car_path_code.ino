// sorter code

#define PRINT_COLOUR                                  // uncomment to turn on output of colour sensor data

#define LEFT_RAMP_SERVO         39                    // GPIO pin for the servo on the left ramp arm
#define RIGHT_RAMP_SERVO        40                    // GPIO pin for the servo on the right ramp arm
#define FUNNEL_SERVO            41                    // GPIO pin for the servo underneath the funnel
#define SLIDE_SERVO             42                    // GPIO pin for the servo attached to the slide

#define PLEFT_RAMP_SERVO         4                   // PWM channel for left ramp servo
#define PRIGHT_RAMP_SERVO        5                   // PWM channel for right ramp servo
#define PFUNNEL_SERVO            6                   // PWM channel for the servo underneath the funnel
#define PSLIDE_SERVO             7                   // PWM channel for the servo attached to the slide


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

// angles servos are set to
const int leftRampServoUp = 180;                     
const int leftRampServoDown = 73;

const int rightRampServoUp = 43;
const int rightRampServoDown = 150;

const int funnelServoStart = 169;
const int funnelServoMiddle = 106;
const int funnelServoEnd = 84;

const int slideServoGreen = 175;
const int slideServoOther = 90;



// total time the servo on the ramp arms will take to go from down to up
unsigned long MOVING_TIME = 2000; // moving time is 2 seconds


 int startTime = 30000; // sorter starts operating at 30 seconds
 int timer = 0;         // timer used to time steps

// boolean variables used to conduct one step at a time
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
 
  
// inital servo positions
 ledcWrite(PLEFT_RAMP_SERVO,degreesToDutyCycle(leftRampServoDown)); 
 ledcWrite(PRIGHT_RAMP_SERVO,degreesToDutyCycle(rightRampServoDown));
 ledcWrite(PFUNNEL_SERVO,degreesToDutyCycle(funnelServoStart));
 ledcWrite(PSLIDE_SERVO,degreesToDutyCycle(slideServoGreen));


// for clock
prevMillis = millis();

}

void loop() {


curMillis = millis();

if ((curMillis-prevMillis)>startTime){ // if the current time is greater than 30 seconds 

  unsigned long progress = millis() - startTime; // time since the start time

  if (progress<MOVING_TIME){ // used to move the servos at a steady pace instead of a jerk motion

   long angleL = map(progress, 0, MOVING_TIME, leftRampServoDown, leftRampServoUp); // angle of left servo dependant on the progress 
   long angleR = map(progress, 0, MOVING_TIME, rightRampServoDown, rightRampServoUp); // angle of right servo dependant on the progress 

   ledcWrite(PLEFT_RAMP_SERVO,degreesToDutyCycle(angleL)); 
   ledcWrite(PRIGHT_RAMP_SERVO,degreesToDutyCycle(angleR));
  }




   if (step1 == false) {
     timer = timer + 1;                          
      if (timer > 150000) {  // timer to stop the system from starting before the beads fall down the funnel                                                   
       ledcWrite(PFUNNEL_SERVO,degreesToDutyCycle(funnelServoMiddle)); // servo slides the bead under the colour sensor

   step1 = true;
   timer = 0;
  }
 }
    


if (step2 == false && step1 == true) {

      uint16_t r, g, b, c;                                // RGBC values from TCS34725
 
  digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));    // turn on onboard LED if switch state is low (on position)                         
  tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values      
  Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c); // used to callibrate sensor

   //Check if the color is green 
     if (g > r && g > b && g > 40 && c < 140) { // paramters to identify green glass beads
       Serial.printf("sensed green\n");
       ledcWrite(PSLIDE_SERVO,degreesToDutyCycle(slideServoGreen)); // servo moves the slides end above the container for green beads
     }
     else{
        Serial.printf("sensed other\n");
       ledcWrite(PSLIDE_SERVO,degreesToDutyCycle(slideServoOther));  // servo moves the slides end away from the container for green beads
     }


  timer = timer + 1;
      if (timer > 400){ // adds delay between the slide moving and dropping the glass bead down the slide
        Serial.printf("%d\n",timer);
       ledcWrite(PFUNNEL_SERVO,degreesToDutyCycle(funnelServoEnd));  // positions the glass bead above the hole to drop the glass bead onto the slide
       step2 = true;
       timer = 0;
      }

   }

   
     if (step3 == false && step2== true) {
     timer = timer + 1;
       if (timer > 150000){
        ledcWrite(PFUNNEL_SERVO,degreesToDutyCycle(funnelServoStart)); // sends the moving frame back under the funnel to collect another glass bead
        timer = 0;
        step1 = false; // resets process
        step2 = false; // resets process
             
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

