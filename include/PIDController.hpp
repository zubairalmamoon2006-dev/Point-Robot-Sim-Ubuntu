#pragma once
#include "Controller.hpp"

class PIDController : public Controller
{
public:
    PIDController(double kp, double ki, double kd, bool antiWindup = true, double integralLimit = 1e9)
        : Kp(kp), Ki(ki), Kd(kd), antiWindup(antiWindup), integralLimit(integralLimit) {}

    Vec2 computeAcceleration(const Robot &robot, const Vec2 &target, double dt) override;
    void reset();

private:
    double Kp, Ki, Kd;
    bool antiWindup;
    double integralLimit;
    Vec2 integral{0.0, 0.0};
};
