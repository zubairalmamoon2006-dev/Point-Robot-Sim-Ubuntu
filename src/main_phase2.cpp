#include "Robot.hpp"
#include "PController.hpp"
#include "PIDController.hpp"
#include <iostream>
#include <iomanip>

static const Vec2 wind{-3.0, 0.0}; // constant, unmodeled disturbance

void runPD(const Vec2 &target, double Kp, double Kd, double dt, int maxSteps)
{
    Robot robot;
    PController controller(Kp, Kd);

    std::cout << "\n--- PD Controller (no Ki) under constant disturbance ---\n";
    for (int step = 0; step < maxSteps; ++step)
    {
        Vec2 a = controller.computeAcceleration(robot, target, dt);
        robot.acceleration = a + wind;
        robot.step(dt, IntegrationScheme::RK4);

        if (step % 100 == 0)
        {
            double error = (target - robot.position).norm();
            std::cout << "t=" << step * dt
                      << "  pos=(" << robot.position.x << ", " << robot.position.y << ")"
                      << "  error=" << error << "\n";
        }
    }
    double finalError = (target - robot.position).norm();
    std::cout << "Final position: (" << robot.position.x << ", " << robot.position.y << ")\n";
    std::cout << "Steady-state error: " << finalError << "\n";
}

void runPID(const Vec2 &target, double Kp, double Ki, double Kd, double dt, int maxSteps)
{
    Robot robot;
    PIDController controller(Kp, Ki, Kd);

    std::cout << "\n--- PID Controller under same disturbance ---\n";
    for (int step = 0; step < maxSteps; ++step)
    {
        Vec2 a = controller.computeAcceleration(robot, target, dt);
        robot.acceleration = a + wind;
        robot.step(dt, IntegrationScheme::RK4);

        if (step % 100 == 0)
        {
            double error = (target - robot.position).norm();
            std::cout << "t=" << step * dt
                      << "  pos=(" << robot.position.x << ", " << robot.position.y << ")"
                      << "  error=" << error << "\n";
        }
    }
    double finalError = (target - robot.position).norm();
    std::cout << "Final position: (" << robot.position.x << ", " << robot.position.y << ")\n";
    std::cout << "Steady-state error: " << finalError << "\n";
}

int main()
{
    std::cout << std::fixed << std::setprecision(3);

    Vec2 target = {10.0, 5.0};
    double dt = 0.01;
    int maxSteps = 3000; // 30s — long enough to reach steady state

    runPD(target, 20.0, 8.944, dt, maxSteps);       // your Phase 1 Trial 4 gains
    runPID(target, 20.0, 8.0, 8.944, dt, maxSteps); // same Kp/Kd, small Ki added
}
