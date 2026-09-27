#pragma once
// placeholder — filled in next
#include "Vec2.hpp"

enum class IntegrationScheme { SemiImplicitEuler, RK4 };

class Robot {
public:
    Vec2 position;
    Vec2 velocity;
    Vec2 acceleration;

    void step(double dt, IntegrationScheme scheme = IntegrationScheme::SemiImplicitEuler);

private:
    void stepSemiImplicitEuler(double dt);
    void stepRK4(double dt);
};

