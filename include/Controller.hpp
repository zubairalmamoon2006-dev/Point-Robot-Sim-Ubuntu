#pragma once
#include "Vec2.hpp"
#include "Robot.hpp"

class Controller {
public:
    virtual Vec2 computeAcceleration(const Robot& robot, const Vec2& target, double dt) = 0;
    virtual ~Controller() = default;
};

