#include <stdint.h>
#include <math.h>


typedef struct{
    float leftEffector;
    float rightEffector; 
} DRONE_EFFECTORS_T;

typedef struct{
    float x_accel;
    float y_accel;
    float x_vel;
    float y_vel;
    float x_pos;
    float y_pos;
    float angle; // straight up is 0, left positive
    float angular_vel;
    float angular_acc;
} DRONE_STATES_T;

typedef struct{
    float mass;
    float inertia;
    float maxThrust;
    float propDist;
} DRONE_AIRFRAME_T;

typedef struct{
    DRONE_AIRFRAME_T airframe;
    DRONE_STATES_T states;
    DRONE_EFFECTORS_T effectors;
    float dt;
} DRONE_T;
DRONE_T drone;

float gravity = 9.81;

uint8_t sim_init(float dt);
void    sim_step(float thr, float steer);
void    calc_accel(void);
float   euler_integrate(float x, float dx_dt);
float   drone_get_x(void);
float   drone_get_y(void);
float   drone_get_angle(void);

uint8_t sim_init(float dt)
{
    drone.dt = dt;
    drone.airframe.mass = 0.25; //250g
    drone.airframe.inertia = 5 * 1e-5;
    drone.airframe.propDist = 0.127/2; //127mm cg to motor center
    drone.airframe.maxThrust = 3; //3N per prop
    return 0;
}

void sim_step(float thr, float steer)
{

    // write cmd inputs into effectors
    drone.effectors.leftEffector  = thr*0.8 - steer*0.01;  
    drone.effectors.rightEffector = thr*0.8 + steer*0.01;

    // use effectors to calc accelerations
    calc_accel();

    //integrate accelerations into velocities
    drone.states.angular_vel = euler_integrate(drone.states.angular_vel, drone.states.angular_acc);
    drone.states.x_vel       = euler_integrate(drone.states.x_vel, drone.states.x_accel);
    drone.states.y_vel       = euler_integrate(drone.states.y_vel, drone.states.y_accel);

    // integrate velocities into positions / attitude
    drone.states.angle = euler_integrate(drone.states.angle, drone.states.angular_vel);
    drone.states.x_pos = euler_integrate(drone.states.x_pos, drone.states.x_vel);
    drone.states.y_pos = euler_integrate(drone.states.y_pos, drone.states.y_vel);
}

void calc_accel()
{
    // get accelerations in bodyframe
    float acc_b = (drone.effectors.leftEffector + drone.effectors.rightEffector) * drone.airframe.maxThrust / drone.airframe.mass;

    // tranlate to x y 
    drone.states.x_accel = -sinf(drone.states.angle) * acc_b;
    drone.states.y_accel =  cosf(drone.states.angle) * acc_b - gravity;

    // angular acceleration (b frame same as xy)
    drone.states.angular_acc = (drone.effectors.rightEffector - drone.effectors.leftEffector) * drone.airframe.maxThrust * drone.airframe.propDist / drone.airframe.inertia;
}

float euler_integrate(float x, float dx_dt)
{
    return x + dx_dt * drone.dt;
}

float drone_get_x()
{
    return drone.states.x_pos;
}

float drone_get_y()
{
    return drone.states.y_pos;
}

float drone_get_angle()
{
    return drone.states.angle;
}