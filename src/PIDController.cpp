#include "PIDController.hpp"

Vec2 PIDController::computeAcceleration(const Robot &robot, const Vec2 &target, double dt)
{
    Vec2 error = target - robot.position;
    Vec2 pTerm = error * Kp;
    Vec2 dTerm = robot.velocity * Kd;

    if (antiWindup)
    {
        // Conditional integration: only accumulate error if the resulting
        // command would NOT be saturated. This is what actually stops windup —
        // it prevents the integral from growing while the robot physically
        // can't follow the extra command anyway.
        Vec2 unsaturated = pTerm + integral * Ki - dTerm;
        if (unsaturated.norm() <= robot.maxAcceleration)
        {
            integral += error * dt;
        }
    }
    else
    {
        integral += error * dt; // naive: always accumulates, saturated or not
    }

    integral = integral.clampedTo(integralLimit); // secondary hard safety net

    Vec2 iTerm = integral * Ki;
    return pTerm + iTerm - dTerm;
}

void PIDController::reset()
{
    integral = {0.0, 0.0};
}