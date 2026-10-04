#include "Robot.hpp"
// placeholder — filled in next
void Robot::step(double dt, IntegrationScheme scheme)
{
    enforceAccelerationLimit(); // clamp BEFORE integrating

    if (scheme == IntegrationScheme::SemiImplicitEuler)
    {
        stepSemiImplicitEuler(dt);
    }
    else
    {
        stepRK4(dt);
    }

    velocity = velocity.clampedTo(maxVelocity); // clamp AFTER integrating
}

void Robot::enforceAccelerationLimit()
{
    acceleration = acceleration.clampedTo(maxAcceleration);
}

void Robot::stepSemiImplicitEuler(double dt)
{
    velocity += acceleration * dt;
    position += velocity * dt;
}

void Robot::stepRK4(double dt)
{
    struct State
    {
        Vec2 pos, vel;
    };

    auto derivative = [&](const State &s) -> State
    {
        return {s.vel, acceleration};
    };

    State s0{position, velocity};

    State k1 = derivative(s0);
    State s1{s0.pos + k1.pos * (dt / 2), s0.vel + k1.vel * (dt / 2)};

    State k2 = derivative(s1);
    State s2{s0.pos + k2.pos * (dt / 2), s0.vel + k2.vel * (dt / 2)};

    State k3 = derivative(s2);
    State s3{s0.pos + k3.pos * dt, s0.vel + k3.vel * dt};

    State k4 = derivative(s3);

    position = s0.pos + (k1.pos + k2.pos * 2.0 + k3.pos * 2.0 + k4.pos) * (dt / 6.0);
    velocity = s0.vel + (k1.vel + k2.vel * 2.0 + k3.vel * 2.0 + k4.vel) * (dt / 6.0);
}
