#define DEBUG_DRIVE_SPEED    1
#define DEBUG_ENCODER_COUNT  1

#include <Adafruit_NeoPixel.h>

// Function declarations
void Indicator();                                                              // for mode/heartbeat on Smart LED
void setMotor(int dir, int pwm, int in1, int in2);     
void ARDUINO_ISR_ATTR encoderISR(void* arg);

// Encoder structure
struct Encoder {
  const int chanA;                                                            // GPIO pin for encoder channel A
  const int chanB;                                                            // GPIO pin for encoder channel B
  long pos;                                                                   // current encoder position
};

// Port pin constants
#define LEFT_MOTOR_A        35                                                 // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B        36                                                 // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A       37                                                 // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B       38                                                 // GPIO38 pin 31 (J38) Motor 2 B
#define ENCODER_LEFT_A      15                                                 // left encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_LEFT_B      16                                                 // left encoder B signal is connected to pin 8 GPIO16 (J16)
#define ENCODER_RIGHT_A     11                                                 // right encoder A signal is connected to pin 19 GPIO11 (J11)
#define ENCODER_RIGHT_B     12                                                 // right encoder B signal is connected to pin 20 GPIO12 (J12)
#define MODE_BUTTON         0                                                  // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH 3                                                  // DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
#define POT_R1              1                                                  // when DIP Switch S1-3 is on, Analog AD0 (pin 39) GPIO1 is connected to Poteniometer R1
#define SMART_LED           21                                                 // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT     1                                                  // number of SMART LEDs in use

// Constants
const int cDisplayUpdate = 100;                                                // update interval for Smart LED in milliseconds
const int cNumMotors = 2;                                                      // number of DC motors
const int cIN1Pin[] = {LEFT_MOTOR_A, RIGHT_MOTOR_A};                           // GPIO pin(s) for INT1
const int cIN1Chan[] = {0, 1};                                                 // PWM channe(s) for INT1
const int c2IN2Pin[] = {LEFT_MOTOR_B, RIGHT_MOTOR_B};                          // GPIO pin(s) for INT2
const int cIN2Chan[] = {2, 3};                                                 // PWM channel(s) for INT2
const int cPWMRes = 8;                                                         // bit resolution for PWM
const int cMinPWM = 150;                                                       // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;                                       // PWM value for maximum speed
const int cPWMFreq = 20000;                                                    // frequency of PWM signal
const int cCountsRev = 1096;                                                   // encoder pulses per motor revolution

// Variables
boolean motorsEnabled = true;                                                  // motors enabled flag
boolean timeUp3sec = false;                                                    // 3 second timer elapsed flag
unsigned char driveSpeed;                                                      // motor drive speed (0-255)
unsigned char driveIndex;                                                      // state index for run mode
unsigned int  modePBDebounce;                                                  // pushbutton debounce timer count
unsigned long timerCount3sec = 0;                                              // 3 second timer count in milliseconds
unsigned long displayTime;                                                     // heartbeat LED update timer
unsigned long previousMicros;                                                  // last microsecond count
unsigned long currentMicros;                                                   // current microsecond count


// step counter
bool step1 = false;
bool step2 = false;
bool step3 = false;
bool step4 = false;

Adafruit_NeoPixel SmartLEDs(SMART_LED_COUNT, SMART_LED, NEO_RGB + NEO_KHZ800);

// smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {5,15,30,45,60,75,90,105,120,135,150,165,180,195,210,225,240,255,
                                      240,225,210,195,180,165,150,135,120,105,90,75,60,45,30,15};

unsigned int  robotModeIndex = 0;                                              // robot operational state                              
unsigned int  modeIndicator[6] = {                                             // colours for different modes
  SmartLEDs.Color(255,0,255),                                                   //   magenta - stop
  SmartLEDs.Color(0,255,0),                                                   //   green - run
  SmartLEDs.Color(0,0,255),                                                   //   blue - empty case
  SmartLEDs.Color(255,255,0),                                                 //   yellow - empty case
  SmartLEDs.Color(0,255,255),                                                 //   cyan - empty case
  SmartLEDs.Color(255,0,0)                                                  //   red - empty case
};                                                                            

Encoder encoder[] = {{ENCODER_LEFT_A, ENCODER_LEFT_B, 0},                      // left encoder, 0 position 
                    {ENCODER_RIGHT_A, ENCODER_RIGHT_B, 0},{ENCODER_RIGHT_C, ENCODER_RIGHT_D, 0},{ENCODER_LEFT_C, ENCODER_LEFT_D, 0}};                   // right encoder, 0 position   

void setup()
{
#if defined DEBUG_DRIVE_SPEED || DEBUG_ENCODER_COUNT
  Serial.begin(115200);
#endif

  // Set up motors and encoders
  for (int k = 0; k < cNumMotors; k++) {
     ledcAttachPin(cIN1Pin[k], cIN1Chan[k]);                                  // attach INT1 GPIO to PWM channel
     ledcSetup(cIN1Chan[k], cPWMFreq, cPWMRes);                               // configure PWM channel frequency and resolution
     ledcAttachPin(c2IN2Pin[k], cIN2Chan[k]);                                 // attach INT2 GPIO to PWM channel
     ledcSetup(cIN2Chan[k], cPWMFreq, cPWMRes);                               // configure PWM channel frequency and resolution
     pinMode(encoder[k].chanA, INPUT);                                        // configure GPIO for encoder channel A input
     pinMode(encoder[k].chanB, INPUT);                                        // configure GPIO for encoder channel B input
     // configure encoder to trigger interrupt with each rising edge on channel A
     attachInterruptArg(encoder[k].chanA, encoderISR, &encoder[k], RISING);
 }

  // Set up SmartLED
  SmartLEDs.begin();                                                          // initialize smart LEDs object (REQUIRED)
  SmartLEDs.clear();                                                          // clear pixel
  SmartLEDs.setPixelColor(0,SmartLEDs.Color(0,0,0));                          // set pixel colors to 'off'
  SmartLEDs.show();                                                           // send the updated pixel colors to the hardware

  pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);                                 // set up motor enable switch with internal pullup
  pinMode(MODE_BUTTON, INPUT_PULLUP);                                         // Set up mode pushbutton
  modePBDebounce = 0;                                                         // reset debounce timer count
}

void loop()
{
  long pos[] = {0, 0};                                                        // current motor positions
  int pot = 0;                                                                // raw ADC value from pot

  // store encoder positions to avoid conflicts with ISR updates
  noInterrupts();                                                             // disable interrupts temporarily while reading
  for (int k = 0; k < cNumMotors; k++) {
     pos[k] = encoder[k].pos;                                                 // read and store current motor position
  }
  interrupts();                                                               // turn interrupts back on




//PATH CODE
  currentMicros = millis();                                                   // get current time in microseconds. Time has been scaled as the timer on the aurdino was broken and started counting too fast
    if (step1 == false) {
     // 3 second timer, counts 3000 milliseconds
     timerCount3sec = timerCount3sec + 1;                                     // increment 3 second timer count
     if (timerCount3sec > 120000) {                                             // if 3 seconds have elapsed
        timerCount3sec = 0;                                                   // reset 3 second timer count
        step1 = true;
        driveIndex = 3;                                                       // go forward
     }
    }
    if (step2 == false && step1 == true) {
     // 3 second timer, counts 3000 milliseconds
     timerCount3sec = timerCount3sec + 1;                                     // increment 3 second timer count
     if (timerCount3sec > 120000) {                                             // if 3 seconds have elapsed
        timerCount2sec = 0;                                                   // reset 3 second timer count
        step2 = true;
        driveIndex = 4;                                                      // go backward

     }
    }
  if (step3 == false && step2 == true) {
      timerCount3sec = timerCount3sec + 1;                                     // increment 3 second timer count
     if (timerCount3sec > 120000 ) {                                             // if 3 seconds have elapsed
        timerCount3sec = 0;                                                   // reset 3 second timer count

        step3 = true;
        driveIndex = 1;                                                       // go right

     }
  }
  if (step4 == false && step3 == true) {
   
     timerCount3sec = timerCount3sec + 1;                                     // increment 3 second timer count
     if (timerCount3sec > 120000) {                                             // if 3 seconds have elapsed
        timerCount3sec = 0;                                                   // reset 3 second timer count
        step4 = true;
        driveIndex = 2;                                                      // go left
     }
  }

  if (step4 == true) {
     timerCount3sec = timerCount3sec + 1;  
     if (timerCount2sec > 120000) {                                             // if 3 seconds have elapsed
        timerCount3sec = 0;                                                   // reset 3 second timer count

        step1 = false;
        step2 = false;
        step3 = false;
        step4 = false;                                                        // reset cycle
     }
  }









     // Mode pushbutton debounce and toggle
     if (!digitalRead(MODE_BUTTON)) {                                         // if pushbutton GPIO goes LOW (nominal push)
        // Start debounce
        if (modePBDebounce <= 25) {                                           // 25 millisecond debounce time
           modePBDebounce = modePBDebounce + 1;                               // increment debounce timer count
           if (modePBDebounce > 25) {                                         // if held for at least 25 mS
              modePBDebounce = 1000;                                          // change debounce timer count to 1 second
           }
        }
        if (modePBDebounce >= 1000) {                                         // maintain 1 second timer count until release
           modePBDebounce = 1000;
        }
     }
     else {                                                                   // pushbutton GPIO goes HIGH (nominal release)
        if(modePBDebounce <= 26) {                                            // if release occurs within debounce interval
           modePBDebounce = 0;                                                // reset debounce timer count
        }
        else {
           modePBDebounce = modePBDebounce + 1;                               // increment debounce timer count
           if(modePBDebounce >= 1025) {                                       // if pushbutton was released for 25 mS
              modePBDebounce = 0;                                             // reset debounce timer count
              robotModeIndex++;                                               // switch to next mode
              robotModeIndex = robotModeIndex & 7;                            // keep mode index between 0 and 7
              timerCount3sec = 0;                                             // reset 3 second timer count

           }
        }
     }
 
     // check if drive motors should be powered
     motorsEnabled = !digitalRead(MOTOR_ENABLE_SWITCH);                       // if SW1-1 is on (low signal), then motors are enabled

     // modes 
     // 0 = Default after power up/reset. Robot is stopped.
     // 1 = Press mode button once to enter.        Run robot
     // 2 = Press mode button twice to enter.       Add your code to do something 
     // 3 = Press mode button three times to enter. Add your code to do something 
     // 4 = Press mode button four times to enter.  Add your code to do something 
     // 5 = Press mode button five times to enter.  Add your code to do something 
     // 6 = Press mode button six times to enter.   Add your code to do something 
     switch(robotModeIndex) {
        case 0: // Robot stopped
           setMotor(0, 0, cIN1Chan[0], cIN2Chan[0]);                          // stop left motor
           setMotor(0, 0, cIN1Chan[1], cIN2Chan[1]);                          // stop right motor
           encoder[0].pos = 0;                                                // clear left encoder
           encoder[1].pos = 0;                                                // clear right encoder
           driveIndex = 0;                                                 
         
           break;

        case 1: // Run robot
                                                            
              // Read pot to update drive motor speed
              pot = analogRead(POT_R1);
              driveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM);
#ifdef DEBUG_DRIVE_SPEED 
              Serial.print(F("Drive Speed: Pot R1 = "));
              Serial.print(pot);
              Serial.print(F(", mapped = "));
              Serial.println(driveSpeed);
#endif
#ifdef DEBUG_ENCODER_COUNT
              Serial.print(F("Left Encoder count = "));
              Serial.print(pos[0]);
              Serial.print(F(" Right Encoder count = "));
              Serial.println(pos[1]);
#endif
              if (motorsEnabled) {                                            // run motors only if enabled
                                                           
                    
                    switch(driveIndex) {                                      // cycle through drive states
                       case 0: // Stop
                          setMotor(0, 0, cIN1Chan[0], cIN2Chan[0]);           // stop left motor
                          setMotor(0, 0, cIN1Chan[1], cIN2Chan[1]);           // stop right motor
                          //driveIndex = 1;                                     // next state: drive forward
                          break;

                       case 1: // Drive left — motors spin in opposite directions as they are opposed by 180 degrees
                          setMotor(-1, driveSpeed, cIN1Chan[0], cIN2Chan[0]);  // left motor forward
                          setMotor(-1, driveSpeed, cIN1Chan[1], cIN2Chan[1]); // right motor reverse (opposite dir from left)
                      
                          break;

                       case 2: // Drive right — motors spin in opposite directions as they are opposed by 180 degrees
                          setMotor(1, driveSpeed, cIN1Chan[0], cIN2Chan[0]); // left motor reverse 
                          setMotor(1, driveSpeed, cIN1Chan[1], cIN2Chan[1]);  // right motor forward (opposite dir from right)
                 
                          break;

                       case 3: // Drive forward - motors spin in same direction
                          setMotor(-1, driveSpeed, cIN1Chan[0], cIN2Chan[0]); // left motor reverse
                          setMotor(1, driveSpeed, cIN1Chan[1], cIN2Chan[1]); // right motor reverse
                         
                          break;

                       case 4: // Drive backward
                          setMotor(1, driveSpeed, cIN1Chan[0], cIN2Chan[0]);  // left motor forward
                          setMotor(-1, driveSpeed, cIN1Chan[1], cIN2Chan[1]);  // right motor forward
        
                          break;
                    }
                 
              }
           
           else {                                                            // stop when motors are disabled
              setMotor(0, 0, cIN1Chan[0], cIN2Chan[0]);                      // stop left motor
              setMotor(0, 0, cIN1Chan[1], cIN2Chan[1]);                      // stop right motor
           }
           break;

        case 2: //add your code to do something 
           robotModeIndex = 0; //  !!!!!!!  remove if using the case
           break;

        case 3: //add your code to do something 
           robotModeIndex = 0; //  !!!!!!!  remove if using the case
           break;

        case 4: //add your code to do something 
           robotModeIndex = 0; //  !!!!!!!  remove if using the case
           break;

        case 5: //add your code to do something 
           robotModeIndex = 0; //  !!!!!!!  remove if using the case
           break;

        case 6: //add your code to do something 
           robotModeIndex = 0; //  !!!!!!!  remove if using the case
           break;
     }
     // Update brightness of heartbeat display on SmartLED
     displayTime++;                                                          // count milliseconds
     if (displayTime > cDisplayUpdate) {                                     // when display update period has passed
        displayTime = 0;                                                     // reset display counter
        LEDBrightnessIndex++;                                                // shift to next brightness level
        if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) {              // if all defined levels have been used
           LEDBrightnessIndex = 0;                                           // reset to starting brightness
        }
        SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);    // set brightness of heartbeat LED
        Indicator();                                                         // update LED
     }
  
}   

// Set colour of Smart LED depending on robot mode (and update brightness)
void Indicator() {
 SmartLEDs.setPixelColor(0, modeIndicator[robotModeIndex]);                  // set pixel colors to = mode 
 SmartLEDs.show();                                                           // send the updated pixel colors to the hardware
}

// send motor control signals, based on direction and pwm (speed)
void setMotor(int dir, int pwm, int in1, int in2) {
  if (dir == 1) {                                                            // forward
     ledcWrite(in1, pwm);
     ledcWrite(in2, 0);
  }
  else if (dir == -1) {                                                      // reverse
     ledcWrite(in1, 0);
     ledcWrite(in2, pwm);
  }
  else {                                                                     // stop
     ledcWrite(in1, 0);
     ledcWrite(in2, 0);
  }
}

// encoder interrupt service routine
// argument is pointer to an encoder structure, which is statically cast to a Encoder structure, allowing multiple
// instances of the encoderISR to be created (1 per encoder)
void ARDUINO_ISR_ATTR encoderISR(void* arg) {
  Encoder* s = static_cast<Encoder*>(arg);                                  // cast pointer to static structure
 
  int b = digitalRead(s->chanB);                                            // read state of channel B
  if (b > 0) {                                                              // high, leading channel A
     s->pos++;                                                              // increase position
  }
  else {                                                                    // low, lagging channel A
     s->pos--;                                                              // decrease position
  }
}






