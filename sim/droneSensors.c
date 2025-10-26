#include <math.h>
#include "droneSensors.h"
#include "nrnd.h"

void accelerometerMeasurement(DRONE_T* drone)
{
    VEC2D_T acc;

    acc.x = sinf(drone->states.angle)*(drone->states.accel.y + GRAVITY) + cosf(drone->states.angle)*drone->states.accel.x;
    acc.y = cosf(drone->states.angle)*(drone->states.accel.y + GRAVITY) - sinf(drone->states.angle)*drone->states.accel.x;

    //ADD NOIS E HERE

    acc.x += nrnd(0, 0.014);
    acc.x += nrnd(0, 0.014);


    //write to drone
    drone->sensors.accelerometer = acc;
   
}

void gyroscopeMeasurement(DRONE_T* drone)
{
    float angularVelocity;
    angularVelocity = drone->states.angular_vel;

    // NOISE HERE 
    angularVelocity += nrnd(0, 0.0038);
    
    drone->sensors.gyroscope = angularVelocity;
}

void GNSSMeasurement_position(DRONE_T* drone)
{
    VEC2D_T GNSS_pos;
    GNSS_pos = drone->states.pos;

    //NOISE HERE
    GNSS_pos.x += nrnd(0,3.3);
    GNSS_pos.y += nrnd(0,5.3);

    drone->sensors.GNSS_pos = GNSS_pos;
}

void GNSSMeasurement_velocity(DRONE_T* drone)
{
    VEC2D_T GNSS_vel;
    GNSS_vel = drone->states.vel;

    //NOISE HERE
    GNSS_vel.x += nrnd(0,0.2);
    GNSS_vel.y += nrnd(0,0.2);

    drone->sensors.GNSS_vel = GNSS_vel;
}