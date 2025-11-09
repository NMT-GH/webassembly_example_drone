#ifndef AIRCRAFT_H
#define AIRCRAFT_H

typedef struct{
    float x;
    float y;
} VEC2D_T;

typedef struct{
    VEC2D_T pos;
    float vel;
    float ang;
    float rotVel;
} STATES2D_T;

typedef struct{
    float maxTurnAcc;
    float maxThrustAcc;
} AIRFRAME2D_T;

typedef struct{
    AIRFRAME2D_T airframe;
    STATES2D_T states;
}AIRCRAFT_T;

#endif