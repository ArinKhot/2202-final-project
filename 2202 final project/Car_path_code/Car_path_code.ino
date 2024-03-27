// double driver code






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

//'back' motors
#define BACK_RIGHT_B            35                                                 
#define BACK_RIGHT_A            36                            
#define BACK_LEFT_A             37                                         
#define BACK_LEFT_B             38                                                 
#define ENCODER_BACK_LEFT_A      4                                                
#define ENCODER_BACK_LEFT_B      5                                               
#define ENCODER_BACK_RIGHT_A     6                                      
#define ENCODER_BACK_RIGHT_B     7           

// 'front' motors
#define FRONT_RIGHT_B            10                                                 
#define FRONT_RIGHT_A            9                            
#define FRONT_LEFT_A             18                                         
#define FRONT_LEFT_B             17                                                 
#define ENCODER_FRONT_LEFT_A     15                                               
#define ENCODER_FRONT_LEFT_B     16                                              
#define ENCODER_FRONT_RIGHT_A    11                                     
#define ENCODER_FRONT_RIGHT_B    12  



#define MODE_BUTTON         0                                                  // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH 3                                                  // DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
#define POT_R1              1                                                  // when DIP Switch S1-3 is on, Analog AD0 (pin 39) GPIO1 is connected to Poteniometer R1
#define SMART_LED           21                                                 // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT     1                                                  // number of SMART LEDs in use


const int cDisplayUpdate = 100;                                                // update interval for Smart LED in milliseconds
const int cNumMotors = 4;                                                      // number of DC motors
const int cIN1Pin[] = {FRONT_LEFT_A, FRONT_RIGHT_A,BACK_LEFT_A,BACK_RIGHT_A};                           // GPIO pin(s) for INT1
const int cIN1Chan[] = {0, 1, 2, 3};                                                 // PWM channe(s) for INT1
const int cIN2Pin[] = {FRONT_LEFT_B, FRONT_RIGHT_B,BACK_LEFT_B,BACK_RIGHT_B};                          // GPIO pin(s) for INT2
const int cIN2Chan[] = {4, 5, 6, 7};                                                 // PWM channel(s) for INT2
const int cPWMRes = 8;                                                         // bit resolution for PWM
const int cMinPWM = 150;                                                       // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;                                       // PWM value for maximum speed
const int cPWMFreq = 20000;                                                    // frequency of PWM signal
const int cCountsRev = 1096;                                                   // encoder pulses per motor revolution

boolean motorsEnabled = true;                                                  // motors enabled flag
unsigned char driveSpeed;                                                      // motor drive speed (0-255)
unsigned char driveIndex;                                                      // state index for run mode
unsigned int  modePBDebounce;                                                  // pushbutton debounce timer count
unsigned long timer = 0;                                                       // 3 second timer count in milliseconds
unsigned long displayTime;                                                     // heartbeat LED update timer
unsigned long previousMillis;                                                  // last millisecond count
unsigned long currentMillis;                                                   // current millisecond count

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

Encoder encoder[] = {{ENCODER_FRONT_LEFT_A, ENCODER_FRONT_LEFT_B, 0}, {ENCODER_FRONT_RIGHT_A, ENCODER_FRONT_RIGHT_B, 0},  // front encoders, 0 position
                    {ENCODER_BACK_LEFT_A, ENCODER_BACK_LEFT_B, 0},{ENCODER_BACK_RIGHT_A, ENCODER_BACK_RIGHT_B, 0}}; // back encoders, 0 position  




void setup()
{
#if defined DEBUG_DRIVE_SPEED || DEBUG_ENCODER_COUNT
  Serial.begin(115200);
#endif

  // Set up motors and encoders
  for (int k = 0; k < cNumMotors; k++) {
     ledcAttachPin(cIN1Pin[k], cIN1Chan[k]);                                  // attach INT1 GPIO to PWM channel
     ledcSetup(cIN1Chan[k], cPWMFreq, cPWMRes);                               // configure PWM channel frequency and resolution
     ledcAttachPin(cIN2Pin[k], cIN2Chan[k]);                                 // attach INT2 GPIO to PWM channel
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
  long pos[] = {0, 0, 0, 0};                                                        // current motor positions
  int pot = 0;                                                                // raw ADC value from pot

  // store encoder positions to avoid conflicts with ISR updates
  noInterrupts();                                                             // disable interrupts temporarily while reading
  for (int k = 0; k < cNumMotors; k++) {
     pos[k] = encoder[k].pos;                                                 // read and store current motor position
  }
  interrupts();                                                               // turn interrupts back on



    if (step1 == false) { 
     timer = timer + 1;                                    
      if (timer > 100) {                                            
          timer = 0;                                                   
          step1 = true;
          driveIndex = 3;                                                      
      }
    }
    if (step2 == false && step1 == true) { 
     timer = timer + 1;                                    
      if (timer > 50000) {                             
         timer = 0;                                                  
         step2 = true;
         driveIndex = 4;                                                     
      }
    }

    if (step3 == false && step2 == true) {
     timer = timer + 1;                                     
      if (timer > 50000 ) {                                    
        timer = 0;                                         
        step3 = true;
        driveIndex = 1;                                                     
      }
    }

    if (step4 == false && step3 == true) {  
     timer = timer + 1;                                    
      if (timer > 50000) {                             
        timer= 0;                                         
        step4 = true;
        driveIndex = 2;                                                   
      }
    }

    if (step4 == true) {
     timer = timer + 1;  
      if (timer> 50000) {                                          
        timer = 0;                                                 
        step1 = false;
        step2 = false;
        step3 = false;
        step4 = false;
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
              timer = 0;                                                      // reset timer count
              step1 = false;
              step2 = false;
              step3 = false;
              step4 = false;
           }
        }
     }

// check if drive motors should be powered
     motorsEnabled = !digitalRead(MOTOR_ENABLE_SWITCH);                       // if SW1-1 is on (low signal), then motors are enabled




switch(robotModeIndex) {
        case 0: // Robot stopped
           setMotor(0, 0, cIN1Chan[0], cIN2Chan[0]);                          // stop front left motor
           setMotor(0, 0, cIN1Chan[1], cIN2Chan[1]);                          // stop front right motor
           setMotor(0, 0, cIN1Chan[2], cIN2Chan[2]);                          // stop back left motor
           setMotor(0, 0, cIN1Chan[3], cIN2Chan[3]);                          // stop back right motor
           encoder[0].pos = 0;                                                // clear front left encoder
           encoder[1].pos = 0;                                                // clear front right encoder
           encoder[2].pos = 0;                                                // clear back left encoder
           encoder[3].pos = 0;                                                // clear back right encoder
           driveIndex = 0;                                                    // set to drive
            step1 = false;
            step2 = false;
            step3 = false;
            step4 = false;
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
                          setMotor(0, 0, cIN1Chan[0], cIN2Chan[0]);                          // stop front left motor
                          setMotor(0, 0, cIN1Chan[1], cIN2Chan[1]);                          // stop front right motor
                          setMotor(0, 0, cIN1Chan[2], cIN2Chan[2]);                          // stop back left motor
                          setMotor(0, 0, cIN1Chan[3], cIN2Chan[3]);                          // stop back right motor
                                                           
                          break;

                       case 1: //turn ?
                          setMotor(-1, driveSpeed, cIN1Chan[0], cIN2Chan[0]);                       
                          setMotor(-1, driveSpeed, cIN1Chan[1], cIN2Chan[1]);                       
                          setMotor(-1, driveSpeed, cIN1Chan[2], cIN2Chan[2]);                     
                          setMotor(-1, driveSpeed, cIN1Chan[3], cIN2Chan[3]);                        
                         
                          break;

                       case 2: // turn ?
                          setMotor(1, driveSpeed, cIN1Chan[0], cIN2Chan[0]);                       
                          setMotor(1, driveSpeed, cIN1Chan[1], cIN2Chan[1]);                  
                          setMotor(1, driveSpeed, cIN1Chan[2], cIN2Chan[2]);                          
                          setMotor(1, driveSpeed, cIN1Chan[3], cIN2Chan[3]);                 
                          
                          break;

                       case 3: //forward
                          setMotor(-1, driveSpeed, cIN1Chan[0], cIN2Chan[0]);                       
                          setMotor(1, driveSpeed, cIN1Chan[1], cIN2Chan[1]);                  
                          setMotor(-1, driveSpeed, cIN1Chan[2], cIN2Chan[2]);                          
                          setMotor(1, driveSpeed, cIN1Chan[3], cIN2Chan[3]);     
                          break;

                       case 4: //backward
                          setMotor(1, driveSpeed, cIN1Chan[0], cIN2Chan[0]);                       
                          setMotor(-1, driveSpeed, cIN1Chan[1], cIN2Chan[1]);                  
                          setMotor(1, driveSpeed, cIN1Chan[2], cIN2Chan[2]);                          
                          setMotor(-1, driveSpeed, cIN1Chan[3], cIN2Chan[3]);     
                       
                          break;
                    }
                 
              }
           
           else {                                                            // stop when motors are disabled
                          setMotor(0, 0, cIN1Chan[0], cIN2Chan[0]);                          // stop front left motor
                          setMotor(0, 0, cIN1Chan[1], cIN2Chan[1]);                          // stop front right motor
                          setMotor(0, 0, cIN1Chan[2], cIN2Chan[2]);                          // stop back left motor
                          setMotor(0, 0, cIN1Chan[3], cIN2Chan[3]);                          // stop back right motor
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











