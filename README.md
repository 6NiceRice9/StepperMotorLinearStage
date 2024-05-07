## Steper motor operates a linear actuator

-------------------- Instruction --------------------
This project enables manual control of a UHV manipulator using a rotary encoder attached to an Arduino. The primary goal is to ensure foolproof and safe operation of a stepper motor.

**Initial State after Arduino Boot:**
- The system enters a hibernation mode in a locked safety state.
- **Safety State:** Stepper motor driver and rotary encoder are turned off.
- **Unlock Mechanism:** Double press the button to activate the rotary encoder, keeping coarse movement disabled. If the button malfunctions or remains pressed, the system remains inactive.

**Operational Modes:**
1. **Fine Movement:**
   - Activated by rotating the rotary encoder after the motor shuts down.
   - Utilizes the AccelStepper library for gentle acceleration and deceleration.
   - The stepper motor remains powered for 3.2 seconds post-rotation to minimize unnecessary power consumption, then shuts down.

2. **Coarse Movement:**
   - During a 3-second window after motor deactivation, press and hold the button to initiate coarse movement.
   - The motor maintains a preset speed and decelerates upon button release.

**Safety Features:**
- **Emergency Stop 1:** During coarse movement, any rotary encoder movement causes the stepper motor to decelerate and lock, regardless of button or encoder status. Reset by pressing the button once.
- **Emergency Stop 2:** During fine movement, rotating the rotary encoder rapidly (10x in succession) triggers a delayed response based on the encoder signal count. To halt the motor immediately, rotate it slightly counterclockwise, which also disables coarse movement for 500 ms.

**Reactivation:**
- After 500 ms, the system re-enables fine movement. Coarse movement can be reactivated by rotating the encoder in the intended direction.

**Manual Adjustment:**
- For precise adjustments, the motor can be rotated manually, taking advantage of the 3-second power-off intervals.

**Recommendations:**
- It's advisable to refine the settings for acceleration and speed to optimize performance based on specific operational requirements.

This configuration ensures that the stepper motor operates efficiently while maintaining high safety standards through controlled mechanisms and emergency interventions.

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
   
