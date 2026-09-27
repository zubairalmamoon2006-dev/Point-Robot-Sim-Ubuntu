#pragma once
#include "Controller.hpp"

class PController : public Controller {
public:
    PController(double kp, double kd) : Kp(kp), Kd(kd) {}

    Vec2 computeAcceleration(const Robot& robot, const Vec2& target, double dt) override;

private:
    double Kp;
    double Kd;
};

