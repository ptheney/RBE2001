#pragma once

class BlueMotor
{
public:
    BlueMotor();
    void setEffort(int effort); // [0,400]
    void moveTo(long position);
    long getPosition();
    void reset();
    void setup();

private:
    void setEffort(int effort, bool clockwise);
    static void ISR_ENCB_EVAL();
    static void ISR_ENCA_EVAL();
    const int tolerance = 3;
    const int PWMOutPin = 11;
    const int AIN2 = 4;
    const int AIN1 = 13;
    const int ENCA = 0;
    const int ENCB = 1;
};