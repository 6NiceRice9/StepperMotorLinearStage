/// -------------------- History ------------------------
/// - rotary enoder works fine
/// - roatating encoder = drives motor --> motor always under power
/// - works with emergeny stop when direction has changed
/// - TURBO MODE NOT SAFE, BUTTON REQUIRED (=pressing button & rotate)!!
/// - all safety features included
/// - bug: after reotation encoder, let the motor go to sleeping mode, press the button once and immeadeatly rotate the encoder -->> uncontrolled acceleration + deacceleration in the encoder rotated direction
/// - bug fixed
/// - prevent post rotation
/// - added full stop during fast rotation (better alternative to slow deacceleration)
/// - had to update with MotorPosition(encoderValue): prevents acceleration immedeatly after unlock (in case the encoder has been rotated between button pushes)
/// - added custom parameter to control speed and times
/// -------------------- Instruction --------------------
/// 1. after powerUP: press the button 2x
/// 2. every 3 seconds after last operation, the motor shuts down
/// 3. after shutting down, the motor has to be moved in thed direction of the acceleration & in the next 3 seconds the button for the acceleratio has to be pressed
/// 4. in case of :button is pressed & motor spins fast, rotation encoder is moved = then safty deacceleration and time delay of 3000 ms & 1 button press
/// 5. adjust the pulse_speed_ms & accel_time_ms !!!
/// ------------------------------------------------------

// ---------------------------------------------------------------------------------------------------------------------
// ---------------------adjust these values ----------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------
// FAST RORATION:
volatile long pulse_speed_ms = 625;              // time to create pulse signal (smaller == FASTER motor rotation) (default = 600)     -->>> maybe: 300 == 400 mm in 60s
volatile long accel_time_ms = 625;              // millicesonds to reach full speed -/- full stop [larger == slower] (default = 625)
volatile long de_accel_time_ms = 625;
long accel_time_reduction_factor = 1;         // [larger == slower] (default = 1)
long de_accel_time_reduction_factor = 1;
int power_off_counter_interval_ms = 2000;   // time until powerOFF motor (default= 3200 ms)
// ---------------------------------------------------------------------------------------------------------------------
// SLOW ROTATION:
int punish_time_ms = 60;                     // response time between directions (>=25ms)
long step_multiplicator = 1;             // multiply encoderValue by this value -->> value > 1, see below
long step_divisor = 2;                  // divide enciderVlaue by this value > 1
long steps = 0;                       // private
unsigned long setMaxSpeed = 20000;      // accelStepper library (default = 1000 ... larger value == faster) == used to control stepper motor rotation during encoder rotation
unsigned int setAcceleration = 20000;  // accelStepper library (default = 1000 ... larger value == faster) == used to control stepper motor rotation during encoder rotation
// ---------------------------------------------------------------------------------------------------------------------
// press x times: to unlock
int button_unlock = 3;    // how many times to press the button to unlock
int false_direction_trigger = 5;   // default = 10 ... smaller == more sensitive to encoder motion
int false_direction_time_slot_ms = 1000;   // after this time, reset trigger to 0. SUM-up the encoder trigger in this time window (smaller slot == faster trigger reset to 0 )
//
// ---------------------------------------------------------------------------------------------------------------------

#include <math.h>         // to use round()

// handling time
extern volatile unsigned long _motor_hibernation_time_ms = 0;


// TurboButton
#define button 7
volatile bool motor_need_acceleration = true;  // true: acceleration required     // false:already accelerated
int emergencyOFF = 1;
int buttonState;
volatile bool hibernation_on = true;
int buttonCount = 0;
int buttonCounts = 0;
extern volatile unsigned long button_time_ms = 0;
extern volatile unsigned long oldButtonTime = 0;

// rotary encoder
#define encoderPinA 2
#define encoderPinB 3

// value encoder
volatile long encoderValue = 0;
volatile long oldEncoderValue = 0;
volatile bool bool_CW_direction = false;   // define direction of the rotation
volatile bool bool_old_direction = false;  // define old direction
volatile int wrongDirectionFlag = 0;         // treshhold for wrong direction trigger (in case of fast encoder rotation)
volatile bool encoder_is_moving = false;     // if encoder is rotated, then == true
extern volatile unsigned long encoderTriggerTimer = 0;

// stepper motor (used only to activate the stepper motor)
#include <AccelStepper.h>
#define stepperPUL 4
#define stepperDIR 5
#define stepperENA 6
volatile bool endOfDeacceleration = true;
volatile bool directoinAccelSpeedDeaccel = -1;    // in case of encoder has changed direction during acceleration, so deacceleration stays in the same direction

AccelStepper StepperMotor(AccelStepper::FULL2WIRE, stepperPUL, stepperDIR);



void setup() {
// Serial.begin(9600);
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
    StepperMotor.setCurrentPosition(encoderValue);              // SET   actual position = 0
    StepperMotor.setMaxSpeed(setMaxSpeed);             // SPEED = Steps / sec
    StepperMotor.setAcceleration(setAcceleration);    // ACCELERATION = Steps /(sec)^2 

}

void loop() 
{
    if (emergencyOFF == 0)    // dont start, if button is pressen during boot or the encoder has been rotated during fast motor speed
    {
        if (hibernation_on == false && digitalRead(button) == 0 && motor_need_acceleration == true)   // not in hibernation mode, button is pressed, acceleration is in process
            {  
                if (encoder_is_moving == true) 
                {   
                    Acceleration();
                }
            }      
        if (digitalRead(button) == 0 && motor_need_acceleration == false)    // Button pressed & acceleration is done
            if(encoder_is_moving == true)                            // emergency stop + delay to prevent acceleration
            {
                noInterrupts();
                // Deacceleration();
                //////// added temporary: instead of Deacceleration();, to have an emergency stop during full speed motor rotation
                StepperMotor.stop();
                motor_need_acceleration = true;    // true == need to accelerate again
                endOfDeacceleration = true;   // true == deacceleration is done
                encoderValue = 0;         // reset encodet value
                encoder_is_moving = true;   // true == encoder has been rotated
                oldEncoderValue = encoderValue;
                StepperMotor.setCurrentPosition(encoderValue);    // overwrite motor position to the actual encoder value
                _motor_hibernation_time_ms = millis();                    // start power off/hybernation counter
                //////// added temporary

                emergencyOFF = 1;
                interrupts();
                delay(1000);
                StepperMotorSTOP();
            }
            else
            {
                KeepSpeed();                                       // else keep motor @ const speed
            }
        if (digitalRead(button) == 1 && motor_need_acceleration == false)    // button released, deacceleration in process
        { 
            if (endOfDeacceleration == false)
            {
                Deacceleration();
            }
        }
        if (digitalRead(button) == 1 && encoder_is_moving == true)   // button released & encoder is rotated == normal usage
        {
            StepperMotorRUN();
            encoderTrigger();
        }
        StepperMotorOFF();
    } 
    else                                                         // emergency section OR power ON section
    {
      StepperMotorOFF();
      buttonState = digitalRead(button);
      if (buttonCounts < button_unlock && buttonCount == 1)
      {
        buttonCounts++;
        buttonCount = 0;    // reset button count;
      }
      else
      {
          if (buttonCounts >= button_unlock)    // after 5x times pressing button, change the flag & return into normal mode
          {
              emergencyOFF = 0;   // 0 == alarm off
              buttonCounts = 0;   // reset button count
              StepperMotor.setCurrentPosition(encoderValue);
              StepperMotorSTOP(); // if encoder has been rotated during unlock time = prevents to seek for new position

          }
      } 
    }
}

void InterruptEncPinA()
{   
    encoder_is_moving = true;
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
    _motor_hibernation_time_ms = millis();   // start power off/hybernation counter
    hibernation_on = false;   // wanking up from hibernation mode
    digitalWrite(stepperENA, LOW);  // power on the stepper motor driver
}

void StepperMotorRUN()  //responsible for slow motor rotation
{   
  if (abs(encoderValue) > 10000)   // prevent overfloat of encoder value
  {
    encoderValue = 0;
    StepperMotor.setCurrentPosition(encoderValue);    // reset the actual motor position
  }
  steps = int(round(abs(encoderValue)*step_multiplicator/step_divisor));    // reduce the amount of steps (int)
  if (bool_CW_direction == true)  // CW motor rotation
  {
    StepperMotor.moveTo(steps);
  }
  else  
   {
    StepperMotor.moveTo(steps * (-1));
   } 
    StepperMotor.run();
}

void StepperMotorSTOP() // stop motor rotation, if  direction of rotation encoder has changed
{
    StepperMotor.stop();
    delay(punish_time_ms);
    encoder_is_moving = false;   // true == encoder has been rotated
    encoderValue = 0;   // prevents the stepper motor to seek for the old position
    StepperMotor.setCurrentPosition(encoderValue);
    bool_old_direction = bool_CW_direction; 
    hibernation_on = true;    // go to hibernation mode   
}
void encoderTrigger()   // counts wrong recognized direction signals
{
    if (bool_CW_direction != bool_old_direction)
    {
        encoderTriggerTimer = millis();
        wrongDirectionFlag++;
        bool_old_direction = !bool_CW_direction;    // new diection is wrong == !bool_CW_direction (opposit of new direction = old direction)
        if (wrongDirectionFlag > false_direction_trigger)    // after n trigger stop rotation
        {   
            wrongDirectionFlag = 0;
            StepperMotorSTOP();
        }
        if (millis() - encoderTriggerTimer > false_direction_time_slot_ms)    // reset the flags if the windows is smaller than the passed by time
        {
            wrongDirectionFlag = 0;
        }  
    }
}
void StepperMotorOFF()
{
  if ((StepperMotor.isRunning() == false) && ((millis() - _motor_hibernation_time_ms) > power_off_counter_interval_ms))
  {
      encoderValue = 0;
      _motor_hibernation_time_ms = millis();    // start power off/hybernation counter
      StepperMotor.stop();
      digitalWrite(stepperENA, HIGH);   // power off the stepper motor driver
      StepperMotor.setCurrentPosition(encoderValue);    // when the encoder has been rotated during button press, prevent acceleration after unlock
      hibernation_on = true;    // go to hibernation mode
  }
}

void Acceleration()   // acceleration step of the motor
{
    directoinAccelSpeedDeaccel = bool_CW_direction;   // keeps acceleration and deacceleratio in the same direction (for the case that the encoder has chagned his direcion during the acceleration time)
    if (directoinAccelSpeedDeaccel == true)   // CW direction
    {
        digitalWrite(stepperDIR, LOW);
        encoder_is_moving = false;
        for (long i = accel_time_ms; i >= 0; i--)   // start CW acceleration
        {
            digitalWrite(stepperPUL, HIGH);
            delayMicroseconds ((pulse_speed_ms + round((i^2) * accel_time_reduction_factor)));
            digitalWrite(stepperPUL, LOW);
            delayMicroseconds ((pulse_speed_ms + round((i^2) * accel_time_reduction_factor)));
        }
        motor_need_acceleration = false;           // false == no acceleration required == full speed rotation of the motor
        endOfDeacceleration = false;    // false == because now, its runs at full speed // (true when the motor stopped)
    }
    if (directoinAccelSpeedDeaccel == false)    // CCW direction
    {
        digitalWrite(stepperDIR, HIGH);
        encoder_is_moving = false;
        for (long i = accel_time_ms; i >= 0; i--)   // start CCW acceleration
        {
            digitalWrite(stepperPUL, HIGH);
            delayMicroseconds ((pulse_speed_ms + round((i^2) * accel_time_reduction_factor)));
            digitalWrite(stepperPUL, LOW);
            delayMicroseconds ((pulse_speed_ms + round((i^2) * accel_time_reduction_factor)));
        }
        motor_need_acceleration = false;           // false == no acceleration required == full speed rotation of the motor
        endOfDeacceleration = false;    // false == because now, its runs at full speed // (true when the motor stopped)
    }
    _motor_hibernation_time_ms = millis();    // start power off/hybernation counter
}
void KeepSpeed()    // keep motor @ const pulse_speed_ms & as long the button is pressed
{
    if ( encoder_is_moving != true && motor_need_acceleration == false)    // when the encoder trigger is activated = stop const. speed
    {
        for (long i = 0; i <= (button_time_ms - oldButtonTime); i++)   // how many steps to drive after acceleration
        {
            digitalWrite(stepperPUL, HIGH);
            delayMicroseconds (pulse_speed_ms);
            digitalWrite(stepperPUL, LOW);
            delayMicroseconds (pulse_speed_ms);
        }
    }
    _motor_hibernation_time_ms = millis();   // reset power OFF timer to avoid power off during holding the button
}   
void Deacceleration()
{
    if (encoder_is_moving == true || digitalRead(button) == 1)
        {
          encoder_is_moving = false;    // reset flag that encoder has been touched
            for (long i = 0; i <= (de_accel_time_ms); i++)   // start deacceleration
            {
              digitalWrite(stepperPUL, HIGH);
              delayMicroseconds ((pulse_speed_ms + round((i^2) * de_accel_time_reduction_factor)));
              digitalWrite(stepperPUL, LOW);
              delayMicroseconds ((pulse_speed_ms + round((i^2) * de_accel_time_reduction_factor)));
            }
        }
    motor_need_acceleration = true;    // true == need to accelerate again
    endOfDeacceleration = true;   // true == deacceleration is done
    encoderValue = 0;         // reset encodet value
    encoder_is_moving = true;   // true == encoder has been rotated
    oldEncoderValue = encoderValue;
    StepperMotor.setCurrentPosition(encoderValue);    // overwrite motor position to the actual encoder value
    _motor_hibernation_time_ms = millis();                    // start power off/hybernation counter
}

void ButtonTimeFalling()
{
  encoder_is_moving = false; // reset encoder trigger (to enter the main loop)
  encoderValue = 0;
  button_time_ms = millis();            // start timer to measure how long the button got pressed
  _motor_hibernation_time_ms = millis();   // start timer for power OFF
}
void ButtonTimeRising()
{
  oldButtonTime = button_time_ms;   // reset button timer
  buttonCount = 1;              // required to unlock the device (first 2 button presses after power off)
}
