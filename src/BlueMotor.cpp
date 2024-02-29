#include <Arduino.h>
#include <BlueMotor.h>
#include <Romi32U4.h>

// Global variables:
long oldValue = 0;
long newValue;
long count = 0;
unsigned time = 0;

BlueMotor::BlueMotor()
{

}

void BlueMotor::setup()
{
    pinMode(PWMOutPin, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    TCCR1A = 0xA8; //0b10101000; //gcl: added OCR1C for adding a third PWM on pin 11
    TCCR1B = 0x11; //0b00010001;
    ICR1 = 400;
    OCR1C = 0;

    digitalRead(5);

    // Attach Encoder A and B ISRs. 
    attachInterrupt(digitalPinToInterrupt(ENCA), ISR_ENCA_EVAL, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB), ISR_ENCB_EVAL, CHANGE);
    reset();
}

long BlueMotor::getPosition()
{
    long tempCount = 0;
   // noInterrupts();
    tempCount = count;
    //interrupts();
    return tempCount;
}

void BlueMotor::reset()
{
    noInterrupts();
    count = 0;
    interrupts();
}

/**
 * @brief Interrupts program to read motor direction to update encoder counts
 * as a qaudrature encoder system. This method is called whenever the Blue Motor
 * Encoder A pin changes states between HIGH and LOW. 
 */
void BlueMotor::ISR_ENCA_EVAL()
{
    // ENCA == 0, ENCB == 1
    // Encoder A is leading after A has changed state, therefore we increment counts for a clockwise turn. 
    if(digitalRead(0) == digitalRead(1)) {
        count++;
    }
    // Encoder B is leading after A has changed state, therefore we decrement counts for a counter-clockwise turn. 
    else {
        count--;
    }
}

/**
 * @brief Interrupts program to read motor direction to update encoder counts
 * as a qaudrature encoder system. This method is called whenever the Blue Motor
 * Encoder B pin changes states between HIGH and LOW. 
 */
void BlueMotor::ISR_ENCB_EVAL()
{
    // ENCA == 0, ENCB == 1
    // Encoder B is leading after B has changed state, therefore we decrement counts for a counter-clockwise turn.
    if(digitalRead(0) == digitalRead(1)) {
        count--;
    }
    // Encoder A is leading after B has changed state, therefore we increment counts for a clockwise turn. 
    else {
        count++;
    }
}

void BlueMotor::setEffort(int effort)
{
    if (effort < 0)
    {
        setEffort(-effort, true);
    }
    else
    {
        setEffort(effort, false);
    }
}

void BlueMotor::setEffort(int effort, bool clockwise)
{
    if (clockwise)
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else
    {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    OCR1C = constrain(effort, 0, 400);
}

/**
 * @brief Moves the blue motor to the specified encoder count position. 
 * The motor leverages a quadrature encoder system such that there are 
 * 540 counts per rotation where clockwise changes in counts are positive
 * and clockwise are negative. 
 * 
 * @note This method uses a tolerance to approximate approaching the specified
 * motor position. The error tolerance is about +/-1%  within being a specified 
 * position. 
 * 
 * @param target (long) : The specified encoder counts to move to. 
 */
void BlueMotor::moveTo(long target)  //Move to this encoder position within the specified
{                                    //tolerance in the header file using proportional control
                                    //then stop
    // Specify tolerance to get a "good enough" approximation when moving the actual motor position to 
    // the specified position. 
    double error_tolerance = 0.01;
    // Error sum for accumulating error for a PI controller. 
    double error_sum = 0; 
    // Loop PI controller until actual motor position in counts is within tolerance. 
    while(getPosition() < (target * (1.0 - error_tolerance)) || getPosition() > (target * (1.0 + error_tolerance))) {
        // Specified PI constants:
        double kp = 0.45, ki = 0.001;

        // Compute controller error, accumulative error, and output effort to set:
        double error = target - getPosition();
        double effort = (error * kp) + (error_sum * ki); 
        setEffort(effort);
        error_sum += error;
    }

    // Turn motor off after reaching desired position. 
    setEffort(0);
}