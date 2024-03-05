#include <Arduino.h>
#include <Romi32U4.h>
// #include <Romi32U4Motors.h>
#include <Chassis.h>
#include <IRdecoder.h>
#include <ir_codes.h>
#include <servo32u4.h>
#include "Timer.h"
#include "BlueMotor.h"
#include "QTRSensors.h"

// Define constants for states
#define IDLE   0
#define GO_TO_ROOF_45_INIT 1
#define GO_TO_ROOF_45 2
#define FACE_ROOF 3
#define REACH_PANEL_1 4
#define TAKE_ROOF_PANEL_1 5
#define GO_TO_BOX 6
#define DELIVER_BOX_PANEL 7
#define LEAVE_BOX_PANEL_1 8
#define TRAVEL_1  9

#define LEAVE_BOX_PANEL_2 10
#define GO_TO_BOX_PANEL_2 11
#define TAKE_ROOF_PANEL_2 12
#define GO_TO_ROOF_25 13
#define LEAVE_ROOF_25 14
#define TRAVEL_2  15

// Create objects
Chassis chassis;
BlueMotor motor;
QTRSensors lineFollower;

// Define Blue Motor Characteristics
#define BLUE_MOTOR_ENCODER_RESOLUTION 540

// Define pins 
#define ECHO_PIN 7 // for ultrasonic rangefinder
#define TRIG_PIN 8 // for ultrasonic rangefinder
#define LINE_PIN_LEFT  A2  // for line following sensor
#define LINE_PIN_RIGHT A3 // for line following sensor
#define SERVO_PIN = 5; // for gripper

const unsigned int linearPotPin = A0; // for gripper

// Gripper Variables
int servoStop = 1490;
int servoJawDown = 100;
int servoJawUp = 2000;
int previousServoMicroseconds = -1;

int linearPotVoltageADC = 512;
int jawOpenPotVoltageADC = 650;
int jawClosedPotVoltageADC = 970;

Servo32U4Pin5 jawServo;  
Timer printTimer(500);

// Ultrasonic Variables
long startTime = 0;
long duration = 0;
long distance = 0; 

int rightEffort = 0;   // right motor effort
int leftEffort = 0;    // left motor effort
int rightDefault = 50; // default speed for right motor
int leftDefault = 50;  // default speed for left motor

// Chassis Variables
int motorEffort = 50;
long driveMillis = 0;
long spinMillis = 0;
unsigned long spinInterval = 1000UL;

// Starting state is idle
int state = IDLE;
int previousState = -1;

bool gripperOpened = false;
bool gripperClosed = false;

// Set up IR decoder
const uint8_t IR_DETECTOR_PIN = 14;
IRDecoder decoder(IR_DETECTOR_PIN);
int16_t keyPress;

// Timer variables for IR
long keyMillis = 0;
unsigned long keyInterval = 1000UL;

double initialCounts = 0;
void initMovement() {
  chassis.resetEncoders();
  initialCounts = (double)(chassis.getLeftEncoderCount() + chassis.getRightEncoderCount()) / 2.0;
}

void setup()
{
  Serial.begin(9600);

  // Initialize chassis and IR decoder
  chassis.init(); 
  decoder.init();
  motor.setup();

  // Attach servo motor for gripper
  jawServo.attach();

  // Stop servo
  jawServo.writeMicroseconds(servoStop);
  initMovement();
}

// Handles key press from IR remote
void handleKeyPress(int16_t keyPress, bool currentlyPressed)
{
  // Print key that was pressed
  //Serial.println("Key: " + String(keyPress));
  //Serial.println(state);

  // When stop is pressed, go to idle unless already in idle
  if (keyPress == STOP_MODE && currentlyPressed)
  {
    // If not already idle, switch to idle
    if (state != IDLE) {
      previousState = state; 
      state = IDLE;
      previousServoMicroseconds = jawServo.getMicroseconds();
      jawServo.writeMicroseconds(servoStop);
    }
    // If already idle, then must be in middle of Task 3
    else if (state == IDLE) {
      state = (previousState == -1) ? GO_TO_ROOF_45_INIT : previousState;
      jawServo.writeMicroseconds(previousServoMicroseconds);
    }
  }
  // When number 1 is pressed, execute task 1
  else if (keyPress == NUM_1 && currentlyPressed)
    state = GO_TO_ROOF_45_INIT;
  // When number 2 is pressed, execute task 2
  else if (keyPress == NUM_2 && currentlyPressed)
    state = LEAVE_BOX_PANEL_1;

  // When number 3 is pressed, execute task 3
  else if (keyPress == NUM_3 && currentlyPressed)
    state = LEAVE_BOX_PANEL_2;  

  // When number 4 is pressed, execute task 4
  else if (keyPress == NUM_4 && currentlyPressed)
    state = TRAVEL_2;

}

// Converts Degrees to Blue Motor Encoder Counts
int getEncoderCountsFromDegrees(double degrees) 
{
  return (degrees / 360.0) * (double)(BLUE_MOTOR_ENCODER_RESOLUTION); 
}  

// Resets gripper to the open position
void openGripper()
{
  // Read potentiometer
  linearPotVoltageADC = analogRead(linearPotPin);
  // Move servo jaw down until specified open state
  if (linearPotVoltageADC < jawOpenPotVoltageADC) {
    gripperOpened = true;
    gripperClosed = false;
    // Stop servo
    jawServo.writeMicroseconds(servoStop);
    return;
  }
  
  gripperOpened = false;
  // Open gripper
  jawServo.writeMicroseconds(servoJawDown);
}

// Closes gripper to grab solar panel
void closeGripper()
{
  // Read potentiometer
  linearPotVoltageADC = analogRead(linearPotPin);

  // Move servo jaw up until specified close state
  if (linearPotVoltageADC > jawClosedPotVoltageADC)
  {
    gripperClosed = true;
    gripperOpened = false;
    // Stop servo
    jawServo.writeMicroseconds(servoStop);
    return;
  }

  gripperClosed = false;
  // Close gripper
  jawServo.writeMicroseconds(servoJawUp);
}

// Read values from line following sensor
void lineFollow(double leftSpeed, double rightSpeed, double inches, bool* done)
{
  // Get back left (A2) and back right (A3) for error:
  // Error is positive to the right, negative to the left
  static int lastError = 0;  // last error for derivative control
  static int error = 0;
  lastError = error; 
  error = analogRead(LINE_PIN_LEFT) - analogRead(LINE_PIN_RIGHT); 
  double currentCounts = (double)(chassis.getLeftEncoderCount() + chassis.getRightEncoderCount()) / 2.0;
  double wheelDiameterInches = 2.75; 
  double deltaCounts = (inches / (wheelDiameterInches * 3.14159265)) * 1440.0; 

  if(abs(currentCounts - initialCounts) < abs(deltaCounts)) {
    // Left bigger right smaller -> + speed left , - speed right
    // Right bigger left smaller -> - speed left, + speed right

    // Use PID controller with Kp and Kd
    // Line Following Variables
    float kp = 0.03, kd = 0.016;      
    int diffSpeed = kp * error + kd * (error - lastError);
    lastError = error; 

    // Move motors accordingly
    chassis.setWheelSpeeds(leftSpeed + diffSpeed, rightSpeed - diffSpeed);
  }
  else {
    *done = true;
    error = 0;
    lastError = 0;
    chassis.setWheelSpeeds(0, 0);
    return;
  }
}

/**
 * @brief Rotates robot about its center as an in-place turn. Counterclockwise is
 * a positive turn and a clockwise is negative where turns are specified in degrees. 
*
 * @param degrees (double) : The specified in-place turn in degrees. 
 */
void pointTurn(double degrees, bool* done)
{
  degrees = degrees * (2.0 * 3.14159265 / 360.0);
  double baseSpeed = 8;//cm/s
  double trackWidthInches = 5.5;
  double wheelDiameterInches = 2.75;

  double arcInches = degrees * (trackWidthInches / 2.0);
  double deltaCounts = (arcInches / (wheelDiameterInches * 3.14159265)) * 1440.0; 

  double sign = degrees / abs(degrees);
  double currentCounts = (double)(abs(chassis.getLeftEncoderCount()) + abs(chassis.getRightEncoderCount())) / 2.0;
  if(abs(currentCounts - initialCounts) < abs(deltaCounts)) {
    chassis.setWheelSpeeds(-sign * baseSpeed, sign * baseSpeed);  
  }
  else {
    *done = true;
    chassis.setMotorEfforts(0, 0);
  }
}

void alignOnIntersection(bool* done) {
  // Get back left (A2) and back right (A3) for error:
  // Error is positive to the right, negative to the left
  static double error = 0; 
  static double prevError = 0;
  static double errorSum = 0;

  prevError = error;
  error = analogRead(LINE_PIN_LEFT) - analogRead(LINE_PIN_RIGHT); 

  // Left bigger right smaller -> + speed left , - speed right
  // Right bigger left smaller -> - speed left, + speed right

  // Use PID controller with Kp and Kd
  double kp = 0.15, ki = 0.002, kd = 0.02;
  int diffSpeed = (kp * error) + (ki * errorSum) + (kd * (error - prevError));
  errorSum += error;

  // Calculate efforts for left and right motors
  rightEffort = - diffSpeed;
  leftEffort = diffSpeed;  

  // Move motors accordingly
  chassis.setMotorEfforts(leftEffort, rightEffort);

  if(error < 10.0) {
    *done = true;
    error = 0; 
    prevError = 0;
    errorSum = 0;
    chassis.setWheelSpeeds(0, 0);
  }
}

void driveForward(double inches, bool* done) {

  double baseSpeed = 5;
  double wheelDiameterInches = 2.75;
  double currentCounts = (double)(chassis.getLeftEncoderCount() + chassis.getRightEncoderCount()) / 2.0;
  double sign = inches / abs(inches);

  double deltaCounts = (inches / (wheelDiameterInches * 3.14159265)) * 1440.0; 

  if(abs(currentCounts - initialCounts) < abs(deltaCounts)) {
    chassis.setWheelSpeeds(-sign * baseSpeed, -sign * baseSpeed);
  }
  else {
    *done = true;
    chassis.setWheelSpeeds(0, 0);
  }
}

void loop()
{
  int16_t readKeyPress = decoder.getKeyCode();
  bool keyPressed = false; 
  // Read IR remote every second
  if (millis() - keyMillis > keyInterval && readKeyPress > -1)
  {
      // Check for a key press on the remote
    keyPress = readKeyPress;
    keyMillis = millis();  
    keyPressed = true;
  }
  //Serial.println(state);
  // Handle key press on IR remote
  if (keyPress > -1) 
      handleKeyPress(keyPress, keyPressed); 

  // State 0: IDLE
  if (state == IDLE)
  {
    chassis.idle();
    return;
  }
  // IR Command 1:
  else if (state == GO_TO_ROOF_45_INIT)
  {
    static bool done = false, preparedArm = false, turned = false; 
    if(done) {
      done = false, preparedArm = false, turned = false;
      state = GO_TO_ROOF_45;
      initMovement();
      delay(1000);
      return;
    }
    // Drive to the house
    openGripper();
    if(!preparedArm) {
      motor.moveToLow(&preparedArm);
    }
    else if(!turned) {
      pointTurn(180, &turned);  
    }  
    else {
      alignOnIntersection(&done);
    }
  }
  else if (state == GO_TO_ROOF_45)
  {
    static bool done = false, traveled = false;
    if(done) {
      done = false, traveled = false; 
      state = FACE_ROOF;
      initMovement();
      delay(500);
      return;
    }

    if(!traveled) {
      lineFollow(13, 13, 9, &traveled);
    }
    else {
      alignOnIntersection(&done);
    }
  }
  else if (state == FACE_ROOF)
  {
    static bool done = false, turnedAround = false, turnedAroundEnded = false; 
    if(done && gripperOpened) {
      done = false, turnedAround = false, turnedAroundEnded = false;
      state = REACH_PANEL_1;
      initMovement();
      delay(500);
      return;
    }

    openGripper();
    if(!turnedAround) {
      pointTurn(180, &turnedAround);
    }
    else if(!turnedAroundEnded) {
      turnedAroundEnded = true;
      initMovement();
    }
    else {
      alignOnIntersection(&done);
    }
  }
  else if (state == REACH_PANEL_1)
  {
    static bool done = false, startClosingGripper = false;
    if(done && gripperClosed) {
      done = false, startClosingGripper = false; 
      state = TAKE_ROOF_PANEL_1;
      initMovement();
      delay(500);
      return;
    }
   
    closeGripper();
    if (!startClosingGripper){
      driveForward(1, &startClosingGripper);
    } 
    else if (gripperClosed) {
      motor.moveTo45Deg(&done);
    }
  }
  else if (state == TAKE_ROOF_PANEL_1)
  {
    static bool done = false, movedForward = false;
    if(done && movedForward) {
      done = false, movedForward = false; 
      state = DELIVER_BOX_PANEL;
      initMovement();
      delay(500);
      return;
    }

    motor.moveToHigh(&done);
    if(!movedForward) {
      driveForward(1, &movedForward);
    }
  }
  else if (state == DELIVER_BOX_PANEL) {
    static bool done = false, leftRoof = false, leftRoofEnded = false, lowerArm = false, turnAround = false, turnAroundEnded = false, aligned = false;
    if(done) {
      done = false, leftRoof = false, leftRoofEnded = false, lowerArm = false, turnAround = false, turnAroundEnded = false, aligned = false;
      state = IDLE;
      initMovement();
      delay(500);
      return;
    }

    if(!leftRoof) {
      lineFollow(13, 13, 8.5, &leftRoof);
    }
    else if(!leftRoofEnded) {
      leftRoofEnded = true;
      initMovement();
      delay(500);
    }
    else if(!turnAround) {
      pointTurn(180, &turnAround);
    }
    else if(!lowerArm) {
      motor.moveToStart(&lowerArm);
    }
    else if(!turnAroundEnded) {
      turnAroundEnded = true;
      initMovement();
      delay(500);
    } 
    else if(!aligned){
      alignOnIntersection(&aligned);
    }
    else {
      openGripper();
      done = gripperOpened;
    }
  }
  // IR Command 2:
  else if (state == LEAVE_BOX_PANEL_1) {
    static bool done = false, leftBoxPanel = false, leftBoxPanelEnded = false, raiseArm = false;   
    if(done) {
      done = false, leftBoxPanel = false, leftBoxPanelEnded = false, raiseArm = false;
      state = TRAVEL_1;
      initMovement();
      delay(500);
      return;
    }

    if(!leftBoxPanel) {
      lineFollow(13, 13, 3, &leftBoxPanel);
    }
    else if(!leftBoxPanelEnded) {
      leftBoxPanelEnded = true;
      initMovement();
      delay(500);
    }
    else if(!raiseArm) {
      motor.moveAboveBox(&raiseArm);
    }
    else if(!done) {
      driveForward(2.8, &done);
    }
  }
  else if (state == TRAVEL_1) {
    static bool done = false, turned = false, turnedEnded = false, traveled = false, traveledEnded = false;
    if(done) {
      done = false, turned = false, turnedEnded = false, traveled = false, traveledEnded = false;
      state = IDLE;
      initMovement();
      delay(500);
      return;
    }

    if(!turned) {
      pointTurn(90, &turned);
    }
    else if(!turnedEnded) {
      turnedEnded = true;
      initMovement();
    }
    else if (!traveled) {
      lineFollow(13, 13, 35.75, &traveled);
    }
    else if(!traveledEnded) {
      traveledEnded = true;
      initMovement();
    }
    else {
      pointTurn(90, &done);
    }
  }
  // IR Command 3:
  else if (state == LEAVE_BOX_PANEL_2) {
    static bool done = false, leftBoxPanel = false, leftBoxPanelEnded = false, loweredArm = false, aligned = false;   
    if(done) {
      done = false, leftBoxPanel = false, leftBoxPanelEnded = false, loweredArm = false, aligned = false; 
      state = GO_TO_BOX_PANEL_2;
      initMovement();
      delay(500);
      return;
    }

    if(!leftBoxPanel) {
      driveForward(8, &leftBoxPanel);
    }
    else if(!leftBoxPanelEnded) {
      leftBoxPanelEnded = true;
      initMovement();
    }
    else if(!loweredArm) {
      motor.moveToStart(&loweredArm);
    }
    else if(!aligned) {
      lineFollow(5, 5, 4, &aligned);
    }
    else {
      pointTurn(180, &done);
    }
  }
  else if (state == GO_TO_BOX_PANEL_2) {
    static bool done = false;
    if(done) {
      done = false;
      state = TAKE_ROOF_PANEL_2;
      initMovement();
      delay(500);
      return;
    }

    openGripper();
    if(!done && gripperOpened) {
      driveForward(1.25, &done);
    }
  }
  else if (state == TAKE_ROOF_PANEL_2) {
    static bool done = false; 
    if(done) {
      done = false;
      state = GO_TO_ROOF_25;
      initMovement();
      delay(500);
      return;
    }

    closeGripper();
    if(gripperClosed) {
      motor.moveToHigh(&done);
    }
  }
  else if (state == GO_TO_ROOF_25) {
    static bool done = false, leftBoxPanel = false, leftBoxPanelEnded = false, turnedAround = false; 
    if(done) {
      done = false, leftBoxPanel = false, leftBoxPanelEnded = false, turnedAround = false;
      state = LEAVE_ROOF_25;
      initMovement();
      delay(500);
      return;
    }

    if(!leftBoxPanel) {
      lineFollow(13, 13, 7.5, &leftBoxPanel);
    }
    else if(!leftBoxPanelEnded) {
      leftBoxPanelEnded = true;
      initMovement();
    }
    else if(!turnedAround) {
      pointTurn(180, &turnedAround);
    }
    else {
      alignOnIntersection(&done);
    }
  }
  else if (state == LEAVE_ROOF_25) {
    static bool done = false, loweredArm = false, leftRoof = false, leftRoofEnded = false; 
    if(done) {
      done = false, loweredArm = false, leftRoof = false, leftRoofEnded = false;
      state = IDLE;
      initMovement();
      delay(500);
      return;
    }

    if(!loweredArm) {
      motor.moveTo25Deg(&loweredArm);
    }
    else if(!gripperOpened) {
      openGripper();
    }
    else if(!leftRoof) {
      lineFollow(13, 13, 9, &leftRoof);
    }
    else if(!leftRoofEnded) {
      leftRoofEnded = true;
      initMovement();
    }
    else {
      pointTurn(90, &done);
    }
  }
  // IR Command 4:
  else if (state == TRAVEL_2) {
    static bool done = false, traveled = false, traveledEnded = false, turned = false, turnedEnded = false; 
    if(done && gripperClosed) {
      done = false, traveled = false, traveledEnded = false, turned = false, turnedEnded = false; 
      state = IDLE; 
      initMovement(); 
      delay(500);
      return;
    }

    closeGripper();
    if(!traveled) {
      lineFollow(13, 13, 35, &traveled);
    }
    else if(!traveledEnded) {
      traveledEnded = true;
      initMovement();
    }
    else if(!turned) {
      pointTurn(-90, &turned);
    }
    else if(!turnedEnded) {
      turnedEnded = true;
      initMovement();
    }
    else {
      motor.moveTo(0, &done);
    }
  }
}