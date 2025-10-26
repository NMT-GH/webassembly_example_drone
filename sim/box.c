#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "drone.h"
#include "droneDynamics.h"
#include "droneController.h"
#include "droneSensors.h"
#include "droneEstimation.h"

DRONE_T drone;

// CONSTANTS ----------
float gravity = 9.81;



uint8_t sim_init(float dt);
void    sim_step(float thr, float steer);
float   drone_get_x(void);
float   drone_get_y(void);
float   drone_get_angle(void);


uint8_t sim_init(float dt)
{
    srand(0);

    drone.dt = dt;
    drone.airframe.mass = 0.25; //250g
    drone.airframe.inertia = 5 * 1e-5;
    drone.airframe.propDist = 0.127/2; //127mm cg to motor center
    drone.airframe.maxThrust = 3; //3N per prop
    return 0;
}

void sim_step(float targetPos_x, float targetPos_y)
{

    VEC2D_T targetPos;
    targetPos.x = targetPos_x;
    targetPos.y = targetPos_y;


    DRONE_EFFECTORS_T effector = dronePositionController(targetPos, &drone);

    droneDynamicStep(&drone, effector.left, effector.right);
    
    accelerometerMeasurement(&drone);
    gyroscopeMeasurement(&drone);
    GNSSMeasurement_position(&drone);
    GNSSMeasurement_velocity(&drone);

    attitudeComplementaryFilter(&drone);


    

}

float drone_get_x()
{
    return drone.states.pos.x;
}

float drone_get_y()
{
    return drone.states.pos.y;
}

float drone_get_angle()
{
    return drone.states.angle;
}