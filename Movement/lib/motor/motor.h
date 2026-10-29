#ifndef Motor_h
#define Motor_h

#include<Arduino.h>

class MotorControl {
    private:
        int ENA, In1, In2;
        int ENB, In3, In4;
    public:
        int LeftMotorSpeed, RightMotorSpeed;
        MotorControl(int pins[6]);
        void Forward(int RightMotorSpeed, int LeftMotorSpeed);
        void TurnLeft(int RightMotorSpeed, int LeftMotorSpeed);
        void TurnRight(int RightMotorSpeed, int LeftMotorSpeed);
        void Stop();
};

#endif