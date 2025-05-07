#ifndef motor_h
#define motor_h
#pragma once

class Motor
{
public:
    Motor(uint8_t speed, uint8_t in1, uint8_t in2);
    void InitializeMotor();
    void MoveForward();
    void MoveBackward();
    void StopMotor();
    void SetSpeed(uint8_t pwm, uint8_t speed);
    uint8_t GetSpeed();
    uint8_t GetIn1();
    uint8_t GetIn2();

private:
    uint8_t speed_;
    uint8_t in1_;
    uint8_t in2_;
};

#endif
