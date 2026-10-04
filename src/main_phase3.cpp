#include "Robot.hpp"
#include "PIDController.hpp"
#include <iostream>
#include <iomanip>
#include <algorithm>

static const Vec2 target{10.0, 5.0};
static const Vec2 wind{-3.0, 0.0};

void runConstrained(double maxAcceleration, double maxVelocity, double dt, int maxSteps)
{
    Robot robot;
    robot.maxAcceleration = maxAcceleration;
    robot.maxVelocity = maxVelocity;

    PIDController controller(20.0, 0.0, 8.944); // Ki=0 — isolate constraint effects, Phase 1 gains

    std::cout << "\n--- Part A: Constrained motion (amax=" << maxAcceleration
              << ", vmax=" << maxVelocity << ") ---\n";

    bool reportedCruise = false;
    for (int step = 0; step < maxSteps; ++step)
    {
        robot.acceleration = controller.computeAcceleration(robot, target, dt);
        robot.step(dt, IntegrationScheme::RK4);

        double error = (target - robot.position).norm();
        double speed = robot.velocity.norm();

        if (!reportedCruise && speed >= maxVelocity - 1e-3)
        {
            std::cout << "First reached max velocity at t=" << (step + 1) * dt << "s\n";
            reportedCruise = true;
        }

        if (step % 20 == 0)
        {
            std::cout << "t=" << step * dt
                      << "  pos=(" << robot.position.x << ", " << robot.position.y << ")"
                      << "  speed=" << speed
                      << "  error=" << error << "\n";
        }

        if (error < 0.05 && speed < 0.05)
        {
            std::cout << "Reached target at t=" << (step + 1) * dt << "s\n";
            break;
        }
    }
}

void runWindupComparison(double maxAcceleration, double dt, int maxSteps)
{
    Robot robotNaive;
    robotNaive.maxAcceleration = maxAcceleration;
    Robot robotAW;
    robotAW.maxAcceleration = maxAcceleration;

    PIDController naive(20.0, 4.0, 8.944, /*antiWindup=*/false);
    PIDController aw(20.0, 4.0, 8.944, /*antiWindup=*/true);

    double naivePeakX = -1e9, awPeakX = -1e9;

    for (int step = 0; step < maxSteps; ++step)
    {
        Vec2 aN = naive.computeAcceleration(robotNaive, target, dt);
        robotNaive.acceleration = aN + wind;
        robotNaive.step(dt, IntegrationScheme::RK4);
        naivePeakX = std::max(naivePeakX, robotNaive.position.x);

        Vec2 aA = aw.computeAcceleration(robotAW, target, dt);
        robotAW.acceleration = aA + wind;
        robotAW.step(dt, IntegrationScheme::RK4);
        awPeakX = std::max(awPeakX, robotAW.position.x);
    }

    std::cout << "\n--- Part B: Windup comparison (amax=" << maxAcceleration << ") ---\n";
    std::cout << "Naive        — peak x=" << naivePeakX
              << "  final=(" << robotNaive.position.x << ", " << robotNaive.position.y << ")\n";
    std::cout << "Anti-windup  — peak x=" << awPeakX
              << "  final=(" << robotAW.position.x << ", " << robotAW.position.y << ")\n";
}

void runClampComparison(double amax, double vmax, double dt, int maxSteps)
{
    Robot unconstrained; // maxAcceleration/maxVelocity stay at default 1e9 — effectively Phase 1 Trial 4
    Robot constrained;
    constrained.maxAcceleration = amax;
    constrained.maxVelocity = vmax;

    PIDController ctrlU(20.0, 0.0, 8.944); // Phase 1 Trial 4 gains, Ki=0, identical on both
    PIDController ctrlC(20.0, 0.0, 8.944);

    std::cout << "\n--- Clamp comparison: unconstrained vs (amax=" << amax
              << ", vmax=" << vmax << ") ---\n";

    for (int step = 0; step < maxSteps; ++step)
    {
        unconstrained.acceleration = ctrlU.computeAcceleration(unconstrained, target, dt);
        unconstrained.step(dt, IntegrationScheme::RK4);

        constrained.acceleration = ctrlC.computeAcceleration(constrained, target, dt);
        constrained.step(dt, IntegrationScheme::RK4);

        if (step % 20 == 0)
        {
            std::cout << "t=" << step * dt
                      << "  | unconstrained: |a|=" << unconstrained.acceleration.norm()
                      << " speed=" << unconstrained.velocity.norm()
                      << "  | constrained: |a|=" << constrained.acceleration.norm()
                      << " speed=" << constrained.velocity.norm()
                      << "\n";
        }
    }
}

int main()
{
    std::cout << std::fixed << std::setprecision(6);

    runConstrained(5.0, 3.0, 0.01, 3000);
    runWindupComparison(5.0, 0.01, 3000);
    runClampComparison(5.0, 3.0, 0.01, 3000);
}
