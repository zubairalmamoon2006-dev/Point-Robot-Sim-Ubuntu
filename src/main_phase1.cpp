#include "Robot.hpp"
#include "PController.hpp"
#include <iostream>
#include <iomanip>

int main()
{
    Robot robot;
    robot.position = {0.0, 0.0};
    robot.velocity = {0.0, 0.0};
    robot.acceleration = {0.0, 0.0};

    Vec2 target = {10.0, 5.0};

    PController controller(20.0, 8.944); // Kp, Kd — tune these

    double dt = 0.01;
    double tolerance = 0.05;
    int maxSteps = 5000;

    std::cout << std::fixed << std::setprecision(3);

    for (int step = 0; step < maxSteps; ++step)
    {
        robot.acceleration = controller.computeAcceleration(robot, target, dt);
        robot.step(dt, IntegrationScheme::RK4);

        double error = (target - robot.position).norm();

        if (step % 20 == 0)
        {
            std::cout << "t=" << step * dt
                      << "  pos=(" << robot.position.x << ", " << robot.position.y << ")"
                      << "  vel=(" << robot.velocity.x << ", " << robot.velocity.y << ")"
                      << "  error=" << error << "\n";
        }

        if (error < tolerance && robot.velocity.norm() < 0.05)
        {
            std::cout << "\nReached target at t=" << step * dt << "s (step " << step << ")\n";
            break;
        }
    }

    std::cout << "\nFinal position: (" << robot.position.x << ", " << robot.position.y << ")\n";
    std::cout << "Final velocity: (" << robot.velocity.x << ", " << robot.velocity.y << ")\n";
}
