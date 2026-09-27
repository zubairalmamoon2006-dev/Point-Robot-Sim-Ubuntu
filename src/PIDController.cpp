#include "PIDController.hpp"

Vec2 PIDController::computeAcceleration(const Robot& robot, const Vec2& target, double dt) {
    Vec2 error = target - robot.position;

    integral += error * dt;

    // Anti-windup: clamp integral magnitude (inactive by default at this Kp/Ki scale)
    if (integral.norm() > integralLimit) {
        integral = integral * (integralLimit / integral.norm());
    }

    Vec2 pTerm = error * Kp;
    Vec2 iTerm = integral * Ki;
    Vec2 dTerm = robot.velocity * Kd;

    return pTerm + iTerm - dTerm;
}

void PIDController::reset() {
    integral = {0.0, 0.0};
}
