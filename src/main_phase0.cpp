#include "Robot.hpp"
#include <iostream>
#include <iomanip>

int main()
{
    Robot robot;
    robot.position = {0.0, 0.0};
    robot.velocity = {0.0, 0.0};
    robot.acceleration = {1.0, 2.0};

    double dt = 0.01;
    double t_total = 2.0;
    int steps = static_cast<int>(t_total / dt);

    for (int i = 0; i < steps; ++i)
    {
        robot.step(dt, IntegrationScheme::SemiImplicitEuler);
    }

    double t = steps * dt;
    Vec2 analyticalPos = {0.5 * robot.acceleration.x * t * t,
                          0.5 * robot.acceleration.y * t * t};
    Vec2 analyticalVel = {robot.acceleration.x * t, robot.acceleration.y * t};

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Simulated  position: (" << robot.position.x << ", " << robot.position.y << ")\n";
    std::cout << "Analytical position: (" << analyticalPos.x << ", " << analyticalPos.y << ")\n";
    std::cout << "Position error norm: " << (robot.position - analyticalPos).norm() << "\n\n";

    std::cout << "Simulated  velocity: (" << robot.velocity.x << ", " << robot.velocity.y << ")\n";
    std::cout << "Analytical velocity: (" << analyticalVel.x << ", " << analyticalVel.y << ")\n";
    std::cout << "Velocity error norm: " << (robot.velocity - analyticalVel).norm() << "\n";
}
