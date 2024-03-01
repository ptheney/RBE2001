#include <Arduino.h>
#include <Romi32U4.h>
#include <Romi32U4Motors.h>
#include <Chassis.h>
#include <IRdecoder.h>
#include <ir_codes.h>
#include <servo32u4.h>
#include "Timer.h"
#include "BlueMotor.h"
#include "QTRSensors.h"

// Define constants for states
#define IDLE   0
#define TASK_1 1
#define TASK_2 2
#define TASK_3 3
#define TASK_4 4
#define TASK_5 5

// Create objects
Chassis chassis;
BlueMotor motor;
QTRSensors lineFollower;

// Define Blue Motor Characteristics
#define BLUE_MOTOR_ENCODER_RESOLUTION 540

// Define pins 
#define ECHO_PIN 7 // for ultrasonic rangefinder
#define TRIG_PIN 8 // for ultrasonic rangefinder
#define LINE_PIN_LEFT  9  // for line following sensor
#define LINE_PIN_RIGHT 3 // for line following sensor
#define SERVO_PIN = 5; // for gripper
int linearPotPin = A0; // for gripper

// Gripper Variables
int servoStop = 1490;
int servoJawDown = 1300;
int servoJawUp = 2000;

int linearPotVoltageADC = 512;
int jawOpenPotVoltageADC = 400;
int jawClosedPotVoltageADC = 1000;

Servo32U4Pin5 jawServo;  
Timer printTimer(500);

// Ultrasonic Variables
long startTime = 0;
long duration = 0;
long distance = 0; 

// Line Following Variables
int lastError = 0;  // last error for derivative control
float KP = 0;       // Kp constant
float KD = 0;       // Kd constant

int rightEffort = 0;   // right motor effort
int leftEffort = 0;    // left motor effort
int rightDefault = 50; // default speed for right motor
int leftDefault = 50;  // default speed for left motor
int rightMax = 400;    // max speed for right motor
int leftMax = 400;     // max speed for left motor
int rightMin = -400;   // min speed for right motor
int leftMin = -400;    // min speed for left motor

// Chassis Variables
int motorEffort = 100;
long driveMillis = 0;
long spinMillis = 0;
unsigned long spinInterval = 1000UL;

// Starting state is idle
int state = IDLE;

// Set up IR decoder
const uint8_t IR_DETECTOR_PIN = 14;
IRDecoder decoder(IR_DETECTOR_PIN);
int16_t keyPress;

// Timer variables for IR
long keyMillis = 0;
unsigned long keyInterval = 1000UL;

// Read distance measurements from ultrasonic sensor
void ultrasonicISR()
{
  // Read echoPin
  duration = micros() - startTime;
  distance = duration / 58; // distance in cm

  // Print distance
  Serial.println(distance);
  delay(100);

  // Clear trigPin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Record start time
  startTime = micros();

  // Set trigPin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
}

void setup()
{
  Serial.begin(9600);

  // Initialize chassis and IR decoder
  chassis.init(); 
  decoder.init();

  // Attach servo motor for gripper
  jawServo.attach();

  // Set up ultrasonic sensor with interrupts
  pinMode(ECHO_PIN, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), ultrasonicISR, CHANGE);

  // Set up line following sensor
  lineFollower.setTypeAnalog();
  lineFollower.setSensorPins((const uint8_t[]) {A2, A3}, 2);
  lineFollower.setEmitterPins(LINE_PIN_LEFT, LINE_PIN_RIGHT);
  lineFollower.calibrate();

  // Clear trigPin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Record start time
  startTime = micros();

  // Set trigPin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Stop servo
  jawServo.writeMicroseconds(servoStop);
}

// Handles key press from IR remote
void handleKeyPress(int16_t keyPress)
{
  // Print key that was pressed
  Serial.println("Key: " + String(keyPress));

  // When stop is pressed, go to idle unless already in idle
  if (keyPress == STOP_MODE)
  {
    // If not already idle, switch to idle
    if (state != IDLE)
      state = IDLE;
    // If already idle, then must be in middle of Task 3
    else
      state = TASK_3;
  }
     
  // When number 1 is pressed, execute task 1
  if (keyPress == NUM_1)
     state = TASK_1;

  // When number 2 is pressed, execute task 2
  if (keyPress == NUM_2)
    state = TASK_2;

  // When number 3 is pressed, execute task 3
  if (keyPress == NUM_3)
    state = TASK_3;  

  // When number 4 is pressed, execute task 4
  if (keyPress == NUM_4)
    state = TASK_4;   

  // When number 5 is pressed, execute task 5
  if (keyPress == NUM_5)
    state = TASK_5; 
}

// Converts Degrees to Blue Motor Encoder Counts
int getEncoderCountsFromDegrees(double degrees) 
{
  return (degrees / 360.0) * (double)(BLUE_MOTOR_ENCODER_RESOLUTION); 
}  

// Drives BlueMotor to move fourbar
void moveFourbar(double degrees)
{
  // Convert degrees to encoder counts by multiplying by 1.5, or 540/360
  motor.moveTo(getEncoderCountsFromDegrees(degrees));
}

// Resets gripper to the open position
void openGripper()
{
  // Read potentiometer
  linearPotVoltageADC = analogRead(linearPotPin);
  
  // Move servo jaw down until specified open state
  while (linearPotVoltageADC > jawOpenPotVoltageADC)
  {
    // Open gripper
    jawServo.writeMicroseconds(servoJawDown);

    // Read potentiometer
    linearPotVoltageADC = analogRead(linearPotPin);

    // Print potentiometer reading
    if (printTimer.isExpired())
    {
      Serial.print("linearPotVoltageADC: ");
      Serial.println(linearPotVoltageADC);
    }
  }
  // Stop servo
  jawServo.writeMicroseconds(servoStop);
}

// Closes gripper to grab solar panel
void closeGripper()
{
  // Read potentiometer
  linearPotVoltageADC = analogRead(linearPotPin);

  // Move servo jaw up until specified close state
  while (linearPotVoltageADC < jawClosedPotVoltageADC)
  {
    // Close gripper
    jawServo.writeMicroseconds(servoJawUp);

    // Read potentiometer
    linearPotVoltageADC = analogRead(linearPotPin);

    // Print potentiometer reading
    if (printTimer.isExpired())
    {
      Serial.print("linearPotVoltageADC: ");
      Serial.println(linearPotVoltageADC);
    }
  }
  // Stop servo
  jawServo.writeMicroseconds(servoStop);
}

// Read values from line following sensor
void lineFollow()
{
  // Get position from sensor
  uint16_t position = lineFollower.readLineBlack((uint16_t *) LINE_PIN_LEFT);

  // Error is positive to the right, negative to the left
  int error = position - 3500; 

  // Use PID controller with Kp and Kd
  int motorSpeed = KP * error + KD * (error - lastError);
  lastError = error;
  
  // Calculate efforts for left and right motors
  rightEffort = rightDefault - motorSpeed;
  leftEffort = leftDefault + motorSpeed;  

  // Keep right motor speed between minimum and maximum
  if (rightEffort < rightMin)
    rightEffort = rightMin;
  else if (rightEffort > rightMax)
    rightEffort = rightMax;

  // Keep left motor speed between minimum and maximum
  if (leftEffort < leftMin)
    leftEffort = leftMin;
  else if (leftEffort > leftMax)
    leftEffort = leftMax;  

  // Move motors accordingly
  chassis.setMotorEfforts(rightEffort, leftEffort);
}

// Drive forward for number of milliseconds (call driveMillis = millis() right before)
void driveForward(int ms)
{
  // Spin for 2000 ms 
  if (millis() - driveMillis < ms)
  {
    // Set both motors to same effort
    chassis.setMotorEfforts(motorEffort, motorEffort);
  }
}

// Rotates chassis 180 degrees (call spinMillis = millis() right before)
void turnAround()
{
  // Spin for 1000 ms
  if (millis() - spinMillis < spinInterval)
  {
    // Robot spins 180 degrees in place
    chassis.setMotorEfforts(motorEffort, -motorEffort);
  }
}

void loop()
{
  openGripper();

  // Check for a key press on the remote
  keyPress = decoder.getKeyCode();

  // If stop is pressed, handle key press immediately
  if (keyPress == STOP_MODE)
    handleKeyPress(keyPress);

  // Read IR remote every second
  if (millis() - keyMillis > keyInterval)
  {
    keyMillis += keyInterval;
  
    // Handle key press on IR remote
    if (keyPress > -1) 
      handleKeyPress(keyPress); 
  }

  // State 0: IDLE
  if (state == IDLE)
  {
    chassis.idle();
  }
  // State 1: Run Task 1
  else if (state == TASK_1)
  {
    // Drive to the house
    driveForward(5000);

    // Pick panel

    // Rotate 180 degrees
    turnAround();

    // Place panel at staging area

    // Stop --> go to state 0: Idle
    state = IDLE;
  }
  // State 2: Task 2
  else if (state == TASK_2)
  {
    // Drive towards the staging area
    // Pick panel
    // Rotate 180 degrees
    // Place panel at house
    // Release panel
    // Stop --> go to state 0: Idle
  }
  // State 3: Task 3
  else if (state == TASK_3)
  {
    // Line follow to other house
    // Stop robot halfway --> go to state 0: Idle
    // Continue line following
  }
  // State 4: Task 4
  else if (state == TASK_4)
  {
    // Drive to the house
    // Pick panel 
    // Rotate 180 degrees
    // Place panel at staging area
    // Stop --> go to state 0: Idle
  }
  // State 5: Task 5
  else if (state == TASK_5)
  {
    // Drive towards the staging area
    // Pick panel
    // Rotate 180 degrees
    // Place panel at house
    // Release panel
    // Stop --> go to state 0: Idle
  }
}