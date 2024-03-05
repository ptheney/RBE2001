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

void BlueMotor::moveToStart(bool* done)
{
    Serial.println("test 1");
    moveTo(START_COUNT, done);
}

void BlueMotor::moveAboveBox(bool* done) 
{
    Serial.println("test 2");
    moveTo(ABOVE_BOX_COUNT, done);
}

void BlueMotor::moveToLow(bool* done)
{
    Serial.println("test 3");
    moveTo(_LOW_COUNT, done);
} 

void BlueMotor::moveTo45Deg(bool* done)
{
    Serial.println("test 4");
    moveTo(_45_DEG_COUNT, done);
}

void BlueMotor::moveToHigh(bool* done) 
{
    Serial.println("test 5");
    moveTo(_HIGH_COUNT, done);
}

void BlueMotor::moveTo25Deg(bool* done)
{
    Serial.println("test 6");
    moveTo(_25_DEG_COUNT, done);
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
        count--;
    }
    // Encoder B is leading after A has changed state, therefore we decrement counts for a counter-clockwise turn. 
    else {
        count++;
    }
}
// 19911 for 45 degree, 2904 for 25 degree, def is -637
// => 993 for 45 deg, 2548 for 25 deg, def is 0
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
        count++;
    }
    // Encoder A is leading after B has changed state, therefore we increment counts for a clockwise turn. 
    else {
        count--;
    }
}

/**
 * @brief Used for driving the motor. Effort ranges from [-400, +400] such that negative will move
 * the four-bar and arm downwards and positive effort will do the opposite. 
 * 
 * @param effort (int) : The specified effort. 
 */
void BlueMotor::setEffort(int effort)
{
    if (effort < 0)
    {
        setEffort(-effort, false);
    }
    else
    {
        setEffort(effort, true);
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
 * @param done (bool*) : Species when the state completes 
 */
void BlueMotor::moveTo(long target, bool* done)  //Move to this encoder position within the specified
{                                    //tolerance in the header file using proportional control
                                    //then stop
    // Specify tolerance to get a "good enough" approximation when moving the actual motor position to 
    // the specified position. 
    double error_tolerance = 0.001;
    // Error sum for accumulating error for a PI controller. 
    static double error_sum = 0; 
    // Find error.
    double error = 0;
    // Loop PI controller until actual motor position in counts is within tolerance. 
    if(getPosition() < (target * (1.0 - error_tolerance)) || getPosition() > (target * (1.0 + error_tolerance))) {
        // Specified PI constants:
        double kp = 0.65, ki = 0.00025;

        // Compute controller error, accumulative error, and output effort to set:
        error = target - getPosition();
        double effort = (error * kp) + (error_sum * ki);
        setEffort(effort);
        error_sum += error;
    }
    else {
        // Turn motor off after reaching desired position. 
        *done = true;
        setEffort(0);
        error_sum = 0;
    }
}