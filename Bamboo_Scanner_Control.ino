#include <AccelStepper.h>
#include <Encoder.h>
#include <Math.h>

//Arduino Pin allocations
//X motor driver wires for pulse(step) and direction
#define directionPinX 22
#define stepPinX 23
//Y motor driver wires for pulse(step) and direction
#define directionPinY 34
#define stepPinY 35
//Head stock limit switch
#define limitSwitch1 21
//Tail stock limit switch
#define limitSwitch2 20
//Camera start operation switch
#define camSwitch 47
//Gantry start operation switch
#define gantrySwitch 46
//Encoder calibration switch
#define encoderSwitch 40
//Encoder pins A and B
#define pinA 18
#define pinB 19
//End Pin allocations

//Motor Definitions using the AccelStepper Library
//The X motor is the gantry motor
//The Y motor is the manipulator motor
AccelStepper stepperX(AccelStepper::DRIVER, stepPinX, directionPinX);
AccelStepper stepperY(AccelStepper::DRIVER, stepPinY, directionPinY);
//End motor defintion

//Encoder Definition using the Encoder library
Encoder myEncoder(pinA, pinB);
//End encoder defintion

//Variable Definitions
//encoder arm length
float probe = 101.6;
//pulses per revolution of the encoder
float ppr = 10000.0;
//angle of encoder arm when zeroed
float zeroAngle = 6.5 * (M_PI/180.0);
//radius if bearing on encoder arm
float bearing = 9.525;
//distance from encoder to center of rotation
float c_dist = 95.0;
//X motor speed
int speed = 600;
//Count cycle for gantry. 1 pass is one count
int count = -1;
//Cycle is how many counts have passed
int cycleCount = 0;
//Distance is how far in pulses the manipulator moves. 800 pulses per revolution of the motor needs 100 pulse increments for 360 degrees at 8 45 degree turns
int distance = 100;
float px = 0;
float slice_distance = 0;
long newPosition = myEncoder.read();
bool encoderZero = false;
float targetDistance;
float mmPerPulse;
//Interrupt booleans for if the limit switch is triggered
volatile bool limitSwitch1Pressed = false;
volatile bool limitSwitch2Pressed = false;
//Debouncing variables for accurate limit switch activation
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long debounceDelay =150;

void setup() {
  //Baud rate of the Arduino Mega 2560
  Serial.begin(115200);
  //Define interrupt switches as pullup
  pinMode(limitSwitch1, INPUT_PULLUP);
  pinMode(limitSwitch2, INPUT_PULLUP);
  //Operational switches just need to be input
  pinMode(camSwitch, INPUT);
  pinMode(gantrySwitch, INPUT);
  //Write them high due to wiring. Will go low when pressed
  digitalWrite(camSwitch, HIGH);
  digitalWrite(gantrySwitch, HIGH);
  digitalWrite(encoderSwitch, HIGH);


  //Values for Gantry Motor
  stepperX.setMaxSpeed(800);
  stepperX.setAcceleration(400);
  stepperX.setSpeed(300);

  //Values for Manipulator Motor
  stepperY.setMaxSpeed(1000);
  stepperY.setAcceleration(600);
  stepperY.setSpeed(1000);

  //Attaching interrupts for the gantry switches
  attachInterrupt(digitalPinToInterrupt(limitSwitch1), limitSwitch1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(limitSwitch2), limitSwitch2ISR, FALLING);
}
float angle = 0;

//FSM states
  enum State {
  CHECK,
  HOMING,
  OPERATION_XB,
  OPERATION_XF,
  OPERATION_Y,
  CAMERA,
  END,
  FINISH
};

//First state is the check state
State currentState = CHECK;


void loop() {
  switch(currentState){
    case CHECK:
      operationCheckState();
      break;  
    case HOMING:
      homingState();
      break;
    case OPERATION_XB:
      operationXBState();   
      break;
    case OPERATION_XF:
      operationXFState();
      break;
    case OPERATION_Y:
      operationYState();
      break;
    case CAMERA:
      operationCamState();
      break;
    case END:
      operationEndState();
      break;
    case FINISH:
      finishState();
      break;
  }
}

//This function has no moving components. In this operation the user will calibrate 
//the encoder first and then press the camera or gantry sequence switch to start the system
void operationCheckState(){
  if(digitalRead(encoderSwitch) == LOW){
      newPosition = 0;
      myEncoder.write(newPosition);
      encoderZero = true;
  }
  if (digitalRead(gantrySwitch) == LOW && encoderZero){
    currentState = HOMING;
  }
  else if (digitalRead(camSwitch) == LOW && encoderZero) {
    currentState = CAMERA;
  }
}

//The gantry will home to the head stock limit switch and set the current motor position to zero
void homingState() {
  count = 0;
    stepperX.setSpeed(speed);
    stepperX.run();
    if (limitSwitch1Pressed) {
      stepperX.stop();
      limitSwitch1Pressed = false; 
      currentState = OPERATION_XF;
      stepperX.setCurrentPosition(0);
    }
}

//Travel along the gantry towards the tail stock. Collect data every 10 motor pulses and stop when hitting the tail stock switch.
void operationXFState() {
  stepperX.setSpeed(-speed);
  stepperX.run(); 
  //reads encoder position
  float newPosition = myEncoder.read();
  //calculates the angle of the encoder
  float theta = (newPosition * (360.0/ppr)) * (M_PI/180);
  //calculates the radius of the bamboo
  float radius = c_dist - ((sin(theta + zeroAngle) * probe) + bearing);
  px = 3.5*sin(theta);
  float slice_distance = abs(stepperX.currentPosition())*mmPerPulse - px + 1.5;
  if (abs(stepperX.currentPosition()) % 10 == 0) { // Adjust sensitivity as needed
    Serial.print(slice_distance, 6); // Print slice distance with 6 decimal places for precision
    Serial.print(", ");
    Serial.print(angle, 2); // Print angle with 2 decimal places for precision
    Serial.print(", ");
    Serial.print(radius, 8); // Print radius with 8 decimal places for precision  
    Serial.print(", ");
    Serial.println(newPosition, 6); // Print radius with 6 decimal places for precision  
  }
   if(digitalRead(limitSwitch2) == LOW) {
    stepperX.stop(); 
    count++;
    currentState = OPERATION_XB;
    limitSwitch2Pressed = false; 
     if(count == 1){
        count = -1;
        cycleCount++;
        currentState = OPERATION_Y;
      }
      else{
        currentState = OPERATION_XB;
      }
  }
}

//Travel along the gantry towards the head stock. Collect data every 10 motor pulses and stop when hitting the head stock switch.
void operationXBState() {
  stepperX.setSpeed(speed);
  stepperX.run();
   //reads encoder position
  float newPosition = myEncoder.read();
  //calculates the angle of the encoder
  float theta = (newPosition * (360.0/ppr)) * (M_PI/180);
  //calculates the radius of the bamboo
  float radius = c_dist - ((sin(theta + zeroAngle) * probe) + bearing);
  px = 3.5*sin(theta);
  float slice_distance = abs(stepperX.currentPosition())*mmPerPulse - px + 1.5;
  if (abs(stepperX.currentPosition()) % 10 == 0) { // Adjust sensitivity as needed
    Serial.print(slice_distance, 6); // Print slice distance with 6 decimal places for precision
    Serial.print(", ");
    Serial.print(angle, 2); // Print angle with 2 decimal places for precision
    Serial.print(", ");
    Serial.print(radius, 8); // Print radius with 8 decimal places for precision  
    Serial.print(", ");
    Serial.println(newPosition, 6); // Print radius with 6 decimal places for precision  
  }
  if(digitalRead(limitSwitch1) == LOW) {  // When switch 1 is pressed, stop the motor
    stepperX.stop();
    currentState = OPERATION_XF;
    count++;
    limitSwitch1Pressed = false;
    stepperX.setCurrentPosition(0);
      if(count == 1){
        count = 0;
        cycleCount++;
        currentState = OPERATION_Y;
      }
      else{
        currentState = OPERATION_XF;
      }
  }
}

//Rotate the motor 45 degrees and move into the next respective state
void operationYState() {
  stepperY.moveTo(distance);
  stepperY.run();
  if (stepperY.distanceToGo() == 0) {
    distance += 100; // Increment distance for the next movement
    stepperY.stop();
    delay(3000);
    Serial.println('P');
    angle+=45;
    delay(11000);
  }
  if(!stepperY.isRunning()){
    if(digitalRead(limitSwitch1 == LOW)){
      currentState = OPERATION_XF; // Move to the forward X operation state
    }
    else if(digitalRead(limitSwitch2 == LOW)){
      currentState = OPERATION_XB; // Move to the forward X operation state
    }
  }
  //if 8 cycles have been counted, move into the end sequence
   if(cycleCount == 8){
     Serial.println('P');
     Serial.print('D');
     angle+=45;
    currentState = END;
  }
}

//Camera only state where the bamboo rotates 45 degrees and the signal for camera capture is sent. NO gantry movement in this section
void operationCamState(){
  stepperY.moveTo(distance); // Move Y motor
  stepperY.run();
  if (stepperY.distanceToGo() == 0) {
    distance += 100; // Increment distance for the next movement
    stepperY.stop();
    delay(3000);
    Serial.println('P');
    angle+=45;
    delay(11000);
    cycleCount++;
  }
  //If 8 cycles have occurred move into the end sequence phase.
  if(cycleCount == 8){
    Serial.println('D');
    currentState = END;
  }
}

//Move the stepper motor off of the last limit switch, priming it for the homing phase after system reset
void operationEndState() {
  stepperX.moveTo(-500);
  stepperX.run();
   if (stepperX.distanceToGo() == 0) {
    stepperX.stop();
    currentState = FINISH;
    }
}

//Stop all motors and finish the system operation
void finishState(){
    stepperX.stop();
    stepperY.stop();
    currentState = FINISH;
}




//Limit switch interrupt code for the gantry motors
void limitSwitch1ISR() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastDebounceTime1 >= debounceDelay) {
    lastDebounceTime1 = currentMillis;
    if (digitalRead(limitSwitch1) == LOW) {
      limitSwitch1Pressed = true;
    }
  }
}
void limitSwitch2ISR() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastDebounceTime2 >= debounceDelay) {
    lastDebounceTime2 = currentMillis;
    if (digitalRead(limitSwitch2) == LOW) {
      limitSwitch2Pressed = true;
    }
  }
}
