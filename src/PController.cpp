#include "PController.hpp"

Vec2 PController::computeAcceleration(const Robot& robot, const Vec2& target, double dt) {
    (void)dt;  // not used yet — this controller reacts to current state only

    Vec2 error = target - robot.position;
    Vec2 pTerm = error * Kp;
    Vec2 dTerm = robot.velocity * Kd;

    return pTerm - dTerm;
}

