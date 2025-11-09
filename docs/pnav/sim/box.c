#include <stdint.h>
#include "aircraft.h"
#include "sim.h"
#include <math.h>
#include <emscripten/emscripten.h>


// CONSTANTS ----------
float gravity = 9.81;
int counter = 0;
int gps_flag = 0;

SIM_T sim;
AIRCRAFT_T ic;
AIRCRAFT_T tg;

uint8_t sim_init(float dt);
void    sim_step(float thr, float steer);
float   drone_get_x(void);
float   drone_get_y(void);
float   drone_get_angle(void);
void targetStep(AIRCRAFT_T *target, VEC2D_T input);
void interceptorStep(AIRCRAFT_T *target, AIRCRAFT_T *interceptor);


uint8_t sim_init(float dt)
{
    //srand(0);

    sim.dt = dt;
    
    tg.airframe.maxThrustAcc = 0;
    tg.airframe.maxTurnAcc = 100; //mDs^2
    tg.states.pos.x = -500; //m
    tg.states.pos.y = 0; //m
    tg.states.vel   = 200; // mDs
    tg.states.ang   = 15 * 3.14 / 180; // mDs

    ic.airframe.maxThrustAcc = 0;
    ic.airframe.maxTurnAcc = 150; //mDs^2
    ic.states.pos.x = 1200; //m
    ic.states.pos.y = -500; //m
    ic.states.vel   = 300; // mDs
    ic.states.ang   = 120* 3.14 / 180; // mDs

    return 0;
}





void sim_step(float leftRight, float frontBack)
{

VEC2D_T input; // inputs should be normalized between -1 and 1
input.x = leftRight; input.y = frontBack;

targetStep(&tg, input);
interceptorStep(&tg, &ic);

if((fabsf(tg.states.pos.x - ic.states.pos.x) < 5) && (fabsf(tg.states.pos.y - ic.states.pos.y) < 5) )
{
    sim_init(sim.dt);
}


}



void targetStep(AIRCRAFT_T *target, VEC2D_T input)
{
    target->states.ang += target->states.rotVel * sim.dt;
    target->states.rotVel = input.x * target->airframe.maxTurnAcc / target->states.vel;

    target->states.pos.x += cosf(target->states.ang) * target->states.vel * sim.dt;
    target->states.pos.y += sinf(target->states.ang) * target->states.vel * sim.dt;
}

void interceptorStep(AIRCRAFT_T *target, AIRCRAFT_T *interceptor)
{


    VEC2D_T ic_tg_pos_dif;
    ic_tg_pos_dif.x = target->states.pos.x - interceptor->states.pos.x;
    ic_tg_pos_dif.y = target->states.pos.y - interceptor->states.pos.y;
    float ic_tg_pos_dif_total = sqrtf(powf(ic_tg_pos_dif.x, 2) + powf(ic_tg_pos_dif.y, 2)); 

    VEC2D_T ic_tg_vel_dif;
    float tg_vel_x = target->states.vel * cosf(target->states.ang);
    float tg_vel_y = target->states.vel * sinf(target->states.ang);
    float ic_vel_x = interceptor->states.vel * cosf(interceptor->states.ang);
    float ic_vel_y = interceptor->states.vel * sinf(interceptor->states.ang);
    ic_tg_vel_dif.x = tg_vel_x - ic_vel_x;
    ic_tg_vel_dif.y = tg_vel_y - ic_vel_y ;
    float ic_tg_vel_dif_total = sqrtf(powf(ic_tg_vel_dif.x, 2) + powf(ic_tg_vel_dif.y, 2)); 

    float ic_tg_ang     = atan2f(ic_tg_pos_dif.y, ic_tg_pos_dif.x);
    float ic_tg_vel_ang = atan2f(ic_tg_vel_dif.y, ic_tg_vel_dif.x);

    float ic_tg_vel_ang_minus_ang = ic_tg_vel_ang - ic_tg_ang;

    float ic_tg_radial_vel  = cosf(ic_tg_vel_ang_minus_ang) * ic_tg_vel_dif_total;
    float ic_tg_tangent_vel = sinf(ic_tg_vel_ang_minus_ang) * ic_tg_vel_dif_total;
    float ic_tg_angular_vel = ic_tg_tangent_vel / ic_tg_pos_dif_total;


    //Proportional Navigation
    float ic_turn_acc = 10 * fabsf(ic_tg_radial_vel) * ic_tg_angular_vel;

    if (ic_turn_acc >  ic.airframe.maxTurnAcc){ic_turn_acc =  ic.airframe.maxTurnAcc;}
    if (ic_turn_acc < -ic.airframe.maxTurnAcc){ic_turn_acc = -ic.airframe.maxTurnAcc;}


    interceptor->states.ang += interceptor->states.rotVel * sim.dt;
    interceptor->states.rotVel = ic_turn_acc / interceptor->states.vel;

    interceptor->states.pos.x += cosf(interceptor->states.ang) * interceptor->states.vel * sim.dt;
    interceptor->states.pos.y += sinf(interceptor->states.ang) * interceptor->states.vel * sim.dt;
}







EMSCRIPTEN_KEEPALIVE
float get_interceptor_pos_x()
{
    return ic.states.pos.x;
}

EMSCRIPTEN_KEEPALIVE
float get_interceptor_pos_y()
{
    return ic.states.pos.y;
}

EMSCRIPTEN_KEEPALIVE
float get_interceptor_kin_ang()
{
    return ic.states.ang;
}


EMSCRIPTEN_KEEPALIVE
float get_target_pos_x()
{
    return tg.states.pos.x;
}

EMSCRIPTEN_KEEPALIVE
float get_target_pos_y()
{
    return tg.states.pos.y;
}

EMSCRIPTEN_KEEPALIVE
float get_target_kin_ang()
{
    return tg.states.ang;
}