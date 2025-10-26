#include "drone.h"
#include "droneDynamics.h"
#include "integrator.h"
#include <math.h>

void calc_accel(DRONE_T *pDrone, float leftCtrlInput, float rightCtrlInput)
{
    // get accelerations in bodyframe
    float acc_b = (leftCtrlInput + rightCtrlInput) * pDrone->airframe.maxThrust / pDrone->airframe.mass;

    // tranlate to x y 
    pDrone->states.accel.x = -sinf(pDrone->states.angle) * acc_b;
    pDrone->states.accel.y =  cosf(pDrone->states.angle) * acc_b - GRAVITY;

    // angular acceleration (b frame same as xy)
    pDrone->states.angular_acc = (rightCtrlInput - leftCtrlInput) * pDrone->airframe.maxThrust * pDrone->airframe.propDist / pDrone->airframe.inertia;
}

void droneDynamicStep(DRONE_T *pDrone, float leftCtrlInput, float rightCtrlInput)
{
    // use effectors to calc accelerations
    calc_accel(pDrone, leftCtrlInput, rightCtrlInput);

    //integrate accelerations into velocities
    pDrone->states.angular_vel = euler_integrate(pDrone->states.angular_vel, pDrone->states.angular_acc, pDrone->dt);
    pDrone->states.vel.x       = euler_integrate(pDrone->states.vel.x, pDrone->states.accel.x, pDrone->dt);
    pDrone->states.vel.y       = euler_integrate(pDrone->states.vel.y, pDrone->states.accel.y, pDrone->dt);

    // integrate velocities into positions / attitude
    pDrone->states.angle = euler_integrate(pDrone->states.angle, pDrone->states.angular_vel, pDrone->dt);
    pDrone->states.pos.x = euler_integrate(pDrone->states.pos.x, pDrone->states.vel.x, pDrone->dt);
    pDrone->states.pos.y = euler_integrate(pDrone->states.pos.y, pDrone->states.vel.y, pDrone->dt);
}