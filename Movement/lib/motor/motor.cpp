#include "Motor.h"

MotorControl::MotorControl(int pins[6]){
    int arr[] = {ENA, ENB, In1, In2, In3, In4};
    for (int i = 0; i < 6; i++){
        arr[i] = pins[i];
    }
    for (int i = 0; i < 6; i++)
    {
        pinMode(arr[i], OUTPUT);
    }
}

void MotorControl::Forward(int RightMotorSpeed, int LeftMotorSpeed){
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
    analogWrite(ENA, RightMotorSpeed);
    digitalWrite(In3, HIGH);
    digitalWrite(In4, LOW);
    analogWrite(ENB, LeftMotorSpeed);
}
void MotorControl::TurnLeft(int RightMotorSpeed, int LeftMotorSpeed){
    digitalWrite(In3, HIGH);
    digitalWrite(In4, LOW);
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
    analogWrite(ENA, LeftMotorSpeed);
    analogWrite(ENB, RightMotorSpeed);
}

void MotorControl::TurnRight(int RightMotorSpeed, int LeftMotorSpeed){
    digitalWrite(In4, HIGH);
    digitalWrite(In3, LOW);
    digitalWrite(In2, LOW);
    digitalWrite(In1, HIGH);
    analogWrite(ENA, LeftMotorSpeed);
    analogWrite(ENB, RightMotorSpeed);
}
void MotorControl::Stop(){
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}