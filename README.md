## Steper motor operates a linear actuator

-------------------- Instruction --------------------

0. This project was made to control a manual UHV manipulator by a rotary encoder, which is resposible for fine a button for coarse positioning. 
0. Since the main prioraty of this arduino code was to have a fool proof & SAFE manual operation of a stepper motor.
1. after booting the arduino, the loop hybernate in a locked safety state

   safety state: stepper motor driver = OFF & rotary encoder = OFF   ----> to unlock: press the button 2x times
   
3. press the button 2x: unlocking rotary encoder (= coarse movement still blocked) ---> in case the button is broken/ keeps pressed = no operation possible

   Now, the rotation of the rotary encoder moves the stepper motor in one of the direction (using AccelStepper library) which accelerate and deaccelerate gently
   
4. after each repositioning of the rotary encoder, the stepper motor keeps powered for 3,2 seconds & shuts down, to prevent unneccessary power consumption.
   (= every 3 seconds after last operation, the stepper motor shuts down)
5. Two stepper motor speeds are avaliable:

  - fine movement: after shutting down, the rotary encoder has to be moved in one of the two directions to have a fine movement of the stepper motor
  
  - coarse movement: in the 3s time slots press and hold the button, the motor accelerate and keeps the predefined speed & deaccelerate @ button release
  
    If the button has been pressed & released in this 3s time slot: the motor accelerated and deaccelerates & again 3s time slot is available
6. Safety features:

  - Emergency stop feature 1: if during the coarse movement, the rotary encoder has been moved: stepper motor deaccelerates and is locked (even the button stay pressed or rotary encoder has moved)
  
   To unlock: press the button 1x.
   
  - Emergency stop feature 2: during fine movement, the rotary encoder could be rotated 10x in a row ... because of the predefined speed, the motor drives the counted amout of signals of encoder (=delayed motor movement)
    
    To stop the motor, rotate slightly in counterclockwise direction. (= motor stops & block the operation for 500 ms & blocks coarse movement till next fine movement)
    
    After 500 ms, its unlocked for fine movement by rotary encoder. --> to unlock coarse movement, rotate in the desired direction
    
7.  If higher precision is required, the motor could be rotated manually, since the motor is powered off every 3s.

8. adjust the acceleration & speed !!!

------------------------------------------------------

# Stepper Motor driven Linear Manipulator

used items:

-(A) "Arduino Nano Every"
https://store.arduino.cc/products/arduino-nano-every

-(B)  "Rotary Encoder 600 ppr 5v-24v From 2 Phase"
https://www.amazon.de/Drehgeber-inklumentierender-2-Phasen-Welle-inklumentierter-Rotationsrate/dp/B08QS2JCF6

-(C)  "ACT 23SSM8440EC Closed Loop Schrittmotor NEMA 23, 1,8 °, 4 A, 2,4 V"
https://www.reichelt.de/closed-loop-schrittmotor-nema-23-1-8-4-a-2-4-v-act-23ssm8440ec-p237919.html

-(D) "DM860T Stepper Motor Driver (input: 18-80VAC or 36-110VDC / output: 2.40A, 3.08A, 3.77A, 4.45A, 5.14A, 5.83A, 6.52A, 7.20A)"
https://www.omc-stepperonline.com/digital-stepper-driver-2-4-7-2a-18-80vac-or-36-110vdc-for-nema-34-motor-dm860t

-(E)  "36V 10A 360W Universal Switching Power Supply"
https://www.amazon.de/dp/B075L68NHB?psc=1&ref=ppx_yo2ov_dt_b_product_details

-(F)   "Button"
# Arduino Pin Overview:

Pin 2(B): rotary encoder (first cable)

Pin 3(B): rotary encoder (second cable)

Pin 4(D): stepper driver PUL/STEP

Pin 5(D): stepper driver DIR/Direction

Pin 6(D): stepper driver ENA/Enable

Pin 7(F): Button


... in process ...
   
