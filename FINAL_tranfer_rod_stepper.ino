/// -------------------- History ------------------------
/// - rotary enoder works fine
/// - roatating encoder = drives motor --> motor always under power
/// - works with emergeny stop when direction has changed
/// - TURBO MODE NOT SAFE, BUTTON REQUIRED (=pressing button & rotate)!!
/// - all safety features included
/// -------------------- Instruction --------------------
/// 1. after powerUP: press the button 2x
/// 2. every 3 seconds after last operation, the motor shuts down
/// 3. after shutting down, the motor has to be moved in thed direction of the acceleration & in the next 3 seconds the button for the acceleratio has to be pressed
/// 4. in case of :button is pressed & motor spins fast, rotation encoder is moved = then safty deacceleration and time delay of 3000 ms & 1 button press
/// 5. adjust the acceleration & speed !!!
/// ------------------------------------------------------

// ---------------------------------------------------------------------------------------------------------------------
// ---------------------adjust these values ----------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------
// handling time
extern volatile unsigned long motorOffFor5Seconds = 0;
int counterInterval = 3200;   // time until powerOFF motor
volatile long accelDeaccerationTime = 625;    // millicesonds to reach full speed -/- full stop [larger == slower] (default = 1000)
volatile long speed = 600;    // time to create pulse signal (smaller == faster motor rotation) (default = 1000)
// ---------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------


// TurboButton
#define button 7
volatile bool acceleration = true;  // true: need to accelerate     // false:already accelerated
volatile long triggerAccel = 100;
int emergencyOFF = 1;
int buttonState;
int buttonCount = 0;
int buttonCounts = 0;
extern volatile unsigned long buttonTime = 0;
extern volatile unsigned long oldButtonTime = 0;

// rotary encoder
#define encoderPinA 2
#define encoderPinB 3

// value encoder
volatile long encoderValue = 0;
volatile long oldEncoderValue = 0;
volatile long diffEncoderValue = 0;
volatile bool bool_CW_direction = false;   // define direction of the rotation
volatile bool bool_old_direction = false;  // define old direction
volatile int wrongDirectionFlag = 0;         // treshhold for wrong direction trigger (in case of fast encoder rotation)
volatile bool triggerEncoder = false;     // if encoder is rotated, then == true
extern volatile unsigned long encoderTriggerTimer = 0;

// stepper motor
#include <AccelStepper.h>
#define stepperPUL 4
#define stepperDIR 5
#define stepperENA 6
unsigned int stepMultiplyFactor = 1;  // multiply encoderValue by this value
unsigned long setMaxSpeed = 1000;     // accelStepper library
unsigned int setAcceleration = 2700;  // accelStepper library
volatile bool endOfDeacceleration = true;
volatile bool directoinAccelSpeedDeaccel = -1;    // in case of encoder has changed direction during acceleration, so deacceleration stays in the same direction

AccelStepper StepperMotor(AccelStepper::FULL2WIRE, stepperPUL, stepperDIR);



void setup() {

    // button
    pinMode(button, INPUT_PULLUP);
    buttonState = digitalRead(button);
    if (buttonState == 0)   // trigger emergency off if the  button is pressen during POWERON
    {
      emergencyOFF = 1;
    }

    // encoder
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);

    // interrupt encoder
    attachInterrupt(digitalPinToInterrupt(encoderPinA), InterruptEncPinA, RISING);
    
    // deacceleration after Button release
    attachInterrupt(digitalPinToInterrupt(button), ButtonTimeFalling, FALLING);
    attachInterrupt(digitalPinToInterrupt(button), ButtonTimeRising, RISING);
    
    // stepper motor  
    pinMode(stepperPUL, OUTPUT);
    pinMode(stepperDIR, OUTPUT);
    pinMode(stepperENA, OUTPUT);
    digitalWrite(stepperENA, HIGH);   // power off the stepper motor driver
    StepperMotor.setCurrentPosition(0);              // SET   actual position = 0
    StepperMotor.setMaxSpeed(setMaxSpeed);             // SPEED = Steps / sec
    StepperMotor.setAcceleration(setAcceleration);    // ACCELERATION = Steps /(sec)^2 

}

void loop() 
{
  // Serial.begin(9600);
    if (emergencyOFF == 0)    // dont start, if button is pressen during boot or the encoder has been rotated during fast motor speed
    {
        if (digitalRead(button) == 0 && acceleration == true)   // button is pressed, acceleration in process
            {  
                if (triggerEncoder == true) 
                {   
                    Acceleration();
                }
            }      
        if (digitalRead(button) == 0 && acceleration == false)    // Button pressed & acceleration is done
            // Serial.println("Button is pressed & acceleration is done");
            if(triggerEncoder == true)                            
            {
                // Serial.println("Emergency stop: encoder has been moved + delay.");        // emergency stop + delay to prevent acceleration
                noInterrupts();
                Deacceleration();
                emergencyOFF = 1;
                interrupts();
                delay(3000);
                StepperMotorSTOP();
                // Serial.println("Emergency stop: done."); 
            }
            else
            {
                // Serial.println("Fast Mode: keep motor @ const. speed"); // else keep motor @ const speed
                KeepSpeed();
            }
        if (digitalRead(button) == 1 && acceleration == false)    // button released, deacceleration in process
        { 
            if (endOfDeacceleration == false)
            {
                // Serial.println("Deacceleration is in process ...");
                Deacceleration();
                // Serial.println("Deacceleration: done");
            }
        }
        if (digitalRead(button) == 1 && triggerEncoder == true)   // button released & encoder is rotated == normal usage
        {
            // Serial.println("Normal Mode");
            StepperMotorRUN();
            encoderTrigger();
        }
        StepperMotorOFF();
    } //// emergency section OR power ON section
    else
    {
      StepperMotorOFF();
      buttonState = digitalRead(button);
      if (buttonCounts < 2 && buttonCount == 1)
      {
        buttonCounts++;
        buttonCount = 0;    // reset button count;
      }
      else
      {
          if (buttonCounts >= 2)    // after 5x times pressing button, change the flag & return into normal mode
          {
              emergencyOFF = 0;   // 0 == alarm off
              buttonCounts = 0;   // reset button count
              StepperMotorSTOP(); // if encoder has been rotated during unlock time = prevents to seek for new position
          }
      } 
    }
}

void InterruptEncPinA()
{   
    triggerEncoder = true;
    if (digitalRead(encoderPinB) == true)
    {
        bool_CW_direction = true;
        encoderValue++;
    }
    else
    {
        bool_CW_direction = false;
        encoderValue--;
    }
    motorOffFor5Seconds = millis();   // start timer, to calculate the power off time, since last action
    digitalWrite(stepperENA, LOW);  // power on the stepper motor driver
}

void StepperMotorRUN()
{   
    StepperMotor.moveTo((encoderValue * stepMultiplyFactor));  // MINUS to reverse rotational direction
    StepperMotor.run();
}

void StepperMotorSTOP() // stop motor rotation, if  direction of rotation encoder has changed
{
    // Serial.println("STOPPED");
    StepperMotor.stop();
    delay(500);
    triggerEncoder = false;   // true == encoder has been rotated
    encoderValue = 0;   // prevents the stepper motor to seek for the old position
    StepperMotor.setCurrentPosition(encoderValue);
    bool_old_direction = bool_CW_direction;    
}
void encoderTrigger()   // counts wrong recognized direction signals
{
    // Serial.println("Encoder Trigger: executed");
    if (bool_CW_direction != bool_old_direction)
    {
        encoderTriggerTimer = millis();
        wrongDirectionFlag++;
        bool_old_direction = !bool_CW_direction;    // new diection is wrong == !bool_CW_direction (opposit of new direction = old direction)
            Serial.print(wrongDirectionFlag);
            Serial.print(" <-- before reset\t");
        if (wrongDirectionFlag > 10)
        {   
            wrongDirectionFlag = 0;
            StepperMotorSTOP();

        }
        if (millis() - encoderTriggerTimer > 1000)
        {
            wrongDirectionFlag = 0;
            Serial.print("after reset -->");
            Serial.println(wrongDirectionFlag);
        }  
    }
}
void StepperMotorOFF()
{
  if ((StepperMotor.isRunning() == false) && ((millis() - motorOffFor5Seconds) > counterInterval))
  {
      encoderValue = 0;
      motorOffFor5Seconds = millis();
      StepperMotor.stop();
      digitalWrite(stepperENA, HIGH);   // power off the stepper motor driver
      StepperMotor.setCurrentPosition(encoderValue);
  }
}

void Acceleration()   // acceleration step of the motor
{
    directoinAccelSpeedDeaccel = bool_CW_direction;   // keeps acceleration and deacceleratio in the same direction (in case encoder has chagned direcio during acceleration)
    if (directoinAccelSpeedDeaccel == true)   // CW direction
    {
        digitalWrite(stepperDIR, LOW);
        triggerEncoder = false;
        for (long i = accelDeaccerationTime; i >= 0; i--)   // start CW acceleration
        {
            digitalWrite(stepperPUL, HIGH);
            delayMicroseconds ((speed +(i^2)));
            digitalWrite(stepperPUL, LOW);
            delayMicroseconds ((speed +(i^2)));
        }
        acceleration = false;           // false == no acceleration required == full speed rotation of the motor
        endOfDeacceleration = false;    // false == because now, its runs at full speed // (true when the motor stopped)
    }
    if (directoinAccelSpeedDeaccel == false)
    {
        digitalWrite(stepperDIR, HIGH);
        triggerEncoder = false;
        for (long i = accelDeaccerationTime; i >= 0; i--)   // start CCW acceleration
        {
            digitalWrite(stepperPUL, HIGH);
            delayMicroseconds ((speed +(i^2)));
            digitalWrite(stepperPUL, LOW);
            delayMicroseconds ((speed +(i^2)));
        }
        acceleration = false;           // false == no acceleration required == full speed rotation of the motor
        endOfDeacceleration = false;    // false == because now, its runs at full speed // (true when the motor stopped)
    }
    motorOffFor5Seconds = millis();
}
void KeepSpeed()    // keep motor @ const speed & as long the button is pressed
{
    if ( triggerEncoder != true && acceleration == false)    // when the encoder trigger is activated = stop const. speed
    {
        for (long i = 0; i <= (buttonTime - oldButtonTime); i++)   // how many steps to drive after acceleration
        {
            digitalWrite(stepperPUL, HIGH);
            delayMicroseconds (speed);
            digitalWrite(stepperPUL, LOW);
            delayMicroseconds (speed);
        }
    }
    motorOffFor5Seconds = millis();   // reset power OFF timer to avoid power off during holding the button
}   
void Deacceleration()
{
    if (triggerEncoder == true || digitalRead(button) == 1)
        {
            for (long i = 0; i <= (accelDeaccerationTime); i++)   // start deacceleration
            {
              digitalWrite(stepperPUL, HIGH);
              delayMicroseconds ((speed +(i^2)));
              digitalWrite(stepperPUL, LOW);
              delayMicroseconds ((speed +(i^2)));
            }
        }
    acceleration = true;    // true == need to accelerate again
    endOfDeacceleration = true;   // true == deacceleration is done
    encoderValue = 0;         // reset encodet value
    triggerEncoder = true;   // true == encoder has been rotated
    oldEncoderValue = encoderValue;
    StepperMotor.setCurrentPosition(encoderValue);    // overwrite motor position to the actual encoder value
    motorOffFor5Seconds = millis();
}

void ButtonTimeFalling()
{
  triggerEncoder = false; // reset encoder trigger (to enter the main loop)
  encoderValue = 0;
  buttonTime = millis();            // start timer to measure how long the button got pressed
  motorOffFor5Seconds = millis();   // start timer for power OFF
}
void ButtonTimeRising()
{
  oldButtonTime = buttonTime;   // reset button timer
  buttonCount = 1;              // required to unlock the device (first 3 button presses after power off)
}