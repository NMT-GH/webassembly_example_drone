#ifndef DRONE_H
#define DRONE_H

typedef struct{
    float x;
    float y;
} VEC2D_T;

typedef struct{
    float left;
    float right; 
} DRONE_EFFECTORS_T;

typedef struct{
    VEC2D_T accel;
    VEC2D_T vel;
    VEC2D_T pos;
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
    VEC2D_T accelerometer;
    float gyroscope;
    VEC2D_T GNSS_pos;
    VEC2D_T GNSS_vel;
} DRONE_SENSORS_T;

typedef struct{
    float angle;
} DRONE_ESTIMATION_T;

typedef struct{
    DRONE_AIRFRAME_T airframe;
    DRONE_STATES_T states;
    DRONE_EFFECTORS_T effectors;
    DRONE_SENSORS_T sensors;
    DRONE_ESTIMATION_T estimation;
    float dt;
} DRONE_T;


#define GRAVITY 9.81

#endif