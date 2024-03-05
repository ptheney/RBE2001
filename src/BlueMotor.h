#pragma once

class BlueMotor
{
public:

    BlueMotor();
    void setEffort(int effort); // [0,400]
    void moveTo(long position, bool* done);
    long getPosition();
    void reset();
    void setup();

    void moveToStart(bool* done);
    void moveAboveBox(bool* done);
    void moveToLow(bool* done); 
    void moveTo45Deg(bool* done); 
    void moveToHigh(bool* done);
    void moveTo25Deg(bool* done);

private:
    void setEffort(int effort, bool clockwise);
    static void ISR_ENCB_EVAL();
    static void ISR_ENCA_EVAL();
    const int tolerance = 3;
    const int PWMOutPin = 11;
    const int AIN2 = 2;
    const int AIN1 = 13;
    const int ENCA = 0;
    const int ENCB = 1;

    const int START_COUNT = 125; 
    const int ABOVE_BOX_COUNT = 1500;
    const int _25_DEG_COUNT = 3990;
    const int _LOW_COUNT = 2754;
    const int _45_DEG_COUNT = 3750;
    const int _HIGH_COUNT = 5250;
};