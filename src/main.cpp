#include <Arduino.h>
#include <Romi32U4.h>
#include <Romi32U4Motors.h>
#include <Chassis.h>
#include <IRdecoder.h>
#include <ir_codes.h>

// Define constants for states
#define IDLE   0
#define TASK_1 1
#define TASK_2 2
#define TASK_3 3
#define TASK_4 4
#define TASK_5 5

// Define pins and variables for sensors
#define ECHO_PIN 7 // for ultrasonic rangefinder
#define TRIG_PIN 8 // for ultrasonic rangefinder
#define LINE_PIN 9 // for line following sensor
long duration;
long distance;

Chassis chassis;
int motorEffort = 100;

// Starting state is idle
int state = IDLE;

// Set up IR decoder
const uint8_t IR_DETECTOR_PIN = 14;
IRDecoder decoder(IR_DETECTOR_PIN);
int16_t keyPress;

void setup()
{
  Serial.begin(9600);

  // Initialize chassis, and IR decoder
  chassis.init(); 
  decoder.init();

  // Set up ultrasonic sensor
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
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

// Read distance measurements from ultrasonic sensor
void readUltrasonic()
{
  // Read from trig pin on ultrasonic sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration/58.2; // distance in centimeters

  // Print distance
  Serial.println(distance);
}

// Read values from line following sensor
void readLineFollower()
{
 
}

void loop()
{
  // Check for a key press on the remote
  keyPress = decoder.getKeyCode();
  
  // Handle key press on IR remote
  if (keyPress > -1) 
    handleKeyPress(keyPress); 

  // State 0: IDLE
  if (state == IDLE)
  {
    chassis.idle();
  }
  // State 1: Run Task 1
  else if (state == TASK_1)
  {
    // Drive to the house
    // Pick panel
    // Rotate 180 degrees
    // Place panel at staging area
    // Stop
  }
  // State 2: Task 2
  else if (state == TASK_2)
  {
    // Drive towards the staging area
    // Pick panel
    // Rotate 180 degrees
    // Place panel at house
    // Release panel
    // Stop
  }
  // State 3: Task 3
  else if (state == TASK_3)
  {
    // Use line following algorithm to navigate to other house
    // Stop robot halfway (switch state to idle)
    // Continue line following
  }
  // State 4: Task 4
  else if (state == TASK_4)
  {
    // Drive to the house
    // Pick panel 
    // Rotate 180 degrees
    // Place panel at staging area
    // Stop
  }
  // State 5: Task 5
  else if (state == TASK_5)
  {
    // Drive towards the staging area
    // Pick panel
    // Rotate 180 degrees
    // Place panel at house
    // Release panel
    // Stop
  }
}