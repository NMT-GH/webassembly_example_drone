#include "integrator.h"

float euler_integrate(float x, float dx_dt, float dt)
{
    return x + dx_dt * dt;
}