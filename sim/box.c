#include <stdint.h>
#include <math.h>

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
    DRONE_AIRFRAME_T airframe;
    DRONE_STATES_T states;
    DRONE_EFFECTORS_T effectors;
    float dt;
} DRONE_T;
DRONE_T drone;

// CONSTANTS ----------
float gravity = 9.81;
float P_GAIN_ANGULAR_VELOCITY = 40;
float P_GAIN_ANGLE = 20;
float P_GAIN_WORLD_VEL_WORLD_ACC = 10;
float P_GAIN_POS_VEL = 3;


uint8_t sim_init(float dt);
void    sim_step(float thr, float steer);
void    calc_accel(float, float);
float   euler_integrate(float x, float dx_dt);
float   drone_get_x(void);
float   drone_get_y(void);
float   drone_get_angle(void);
void droneDynamicStep(float , float );
DRONE_EFFECTORS_T dronePositionController(VEC2D_T );
DRONE_EFFECTORS_T forceMomentController(float , float );
float angularVelocityController(float , float );
float attitudeController(float , float );
float targetWorldAccToTargetAtt(VEC2D_T );
float targetWorldAccToTargetAcc(VEC2D_T, float, float);
VEC2D_T targetWorldVelToTargetWorldAcc(VEC2D_T , VEC2D_T );
VEC2D_T targetPosToTargetVelocity(VEC2D_T , VEC2D_T );

uint8_t sim_init(float dt)
{
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


    DRONE_EFFECTORS_T effector = dronePositionController(targetPos);
    droneDynamicStep(effector.left, effector.right);

}

void calc_accel(float leftCtrlInput, float rightCtrlInput)
{
    // get accelerations in bodyframe
    float acc_b = (leftCtrlInput + rightCtrlInput) * drone.airframe.maxThrust / drone.airframe.mass;

    // tranlate to x y 
    drone.states.accel.x = -sinf(drone.states.angle) * acc_b;
    drone.states.accel.y =  cosf(drone.states.angle) * acc_b - gravity;

    // angular acceleration (b frame same as xy)
    drone.states.angular_acc = (rightCtrlInput - leftCtrlInput) * drone.airframe.maxThrust * drone.airframe.propDist / drone.airframe.inertia;
}

void droneDynamicStep(float leftCtrlInput, float rightCtrlInput)
{
    // use effectors to calc accelerations
    calc_accel(leftCtrlInput, rightCtrlInput);

    //integrate accelerations into velocities
    drone.states.angular_vel = euler_integrate(drone.states.angular_vel, drone.states.angular_acc);
    drone.states.vel.x       = euler_integrate(drone.states.vel.x, drone.states.accel.x);
    drone.states.vel.y       = euler_integrate(drone.states.vel.y, drone.states.accel.y);

    // integrate velocities into positions / attitude
    drone.states.angle = euler_integrate(drone.states.angle, drone.states.angular_vel);
    drone.states.pos.x = euler_integrate(drone.states.pos.x, drone.states.vel.x);
    drone.states.pos.y = euler_integrate(drone.states.pos.y, drone.states.vel.y);
}

DRONE_EFFECTORS_T dronePositionController(VEC2D_T targetPos)
{
    VEC2D_T targetVelocity     = targetPosToTargetVelocity(targetPos, drone.states.pos);
    VEC2D_T targetAcceleration = targetWorldVelToTargetWorldAcc(targetVelocity, drone.states.vel);
    float targetAttitude       = targetWorldAccToTargetAtt(targetAcceleration);
    float targetTotalAcc       = targetWorldAccToTargetAcc(targetAcceleration, targetAttitude, drone.states.angle);

    float targetAngularVel     = attitudeController(targetAttitude, drone.states.angle);
    float targetAngularAcc     = angularVelocityController(targetAngularVel, drone.states.angular_vel);

    DRONE_EFFECTORS_T effector = forceMomentController(targetTotalAcc, targetAngularAcc);
    return effector;
}

// DRONE_EFFECTORS_T dronePositionController(VEC2D_T targetPos)
// {


//     float targetAngularVel     = attitudeController(targetPos.y*(45*3.14/180), drone.states.angle);
//     float targetAngularAcc     = angularVelocityController(targetAngularVel, drone.states.angular_vel);

//     DRONE_EFFECTORS_T effector = forceMomentController(gravity * ( 1/cosf(drone.states.angle) + targetPos.x),  targetAngularAcc);
//     return effector;
// }

DRONE_EFFECTORS_T forceMomentController(float targetAccel, float targetAngularAccel)
{

    DRONE_EFFECTORS_T effector; 

    // summedThrustInput * maxThrust / droneMass = targetAccel
    float summedThrustInput = targetAccel * drone.airframe.mass / drone.airframe.maxThrust;
    
    // differenceThrustOutput * maxThrust * propdist / intertia = angualr_acc
    float differenceThrustOutput = targetAngularAccel * drone.airframe.inertia / (drone.airframe.maxThrust * drone.airframe.propDist);

    effector.left  = 0.5 * (summedThrustInput - differenceThrustOutput);
    effector.right = 0.5 * (summedThrustInput + differenceThrustOutput);

    if (effector.left  > 1){effector.left  = 1;}
    if (effector.left  < 0){effector.left  = 0;}
    if (effector.right > 1){effector.right = 1;}
    if (effector.right < 0){effector.right = 0;}

    return effector;
}

float angularVelocityController(float targetAngularVelocity, float currentAngularVelocity)
{
    // proportional control for now
    float angVelError = targetAngularVelocity - currentAngularVelocity;

    return angVelError * P_GAIN_ANGULAR_VELOCITY;
}

float attitudeController(float targetAngle, float currentAngle)
{
    // proportional control for now
    float angleError = targetAngle - currentAngle;

    return angleError * P_GAIN_ANGLE;
}

float targetWorldAccToTargetAtt(VEC2D_T targetAcc)
{
    return atanf( -(targetAcc.x / (targetAcc.y + gravity)) );
}

float targetWorldAccToTargetAcc(VEC2D_T targetAcc, float targetAtt, float currentAtt)
{

    float totalAcc = sqrtf( powf(targetAcc.x,2) + powf(targetAcc.y + gravity, 2) );

    float alignmentFactor = cosf(targetAtt - currentAtt);
    

    return totalAcc * alignmentFactor;
}

VEC2D_T targetWorldVelToTargetWorldAcc(VEC2D_T targetVel, VEC2D_T currentVel)
{
    VEC2D_T errorVelocity;
    VEC2D_T targetAcceleration;

    errorVelocity.x = targetVel.x - currentVel.x;
    errorVelocity.y = targetVel.y - currentVel.y;

    targetAcceleration.x = errorVelocity.x * P_GAIN_WORLD_VEL_WORLD_ACC;
    targetAcceleration.y = errorVelocity.y * P_GAIN_WORLD_VEL_WORLD_ACC;

    if (targetAcceleration.y < -(gravity * 0.9)){targetAcceleration.y = -(gravity * 0.9);} 

    return targetAcceleration;
}

VEC2D_T targetPosToTargetVelocity(VEC2D_T targetPos, VEC2D_T currentPos)
{
    VEC2D_T errorPosition;
    VEC2D_T targetVelocity;

    errorPosition.x = targetPos.x - currentPos.x;
    errorPosition.y = targetPos.y - currentPos.y;

    targetVelocity.x = errorPosition.x * P_GAIN_POS_VEL;
    targetVelocity.y = errorPosition.y * P_GAIN_POS_VEL;

    return targetVelocity;
}

float euler_integrate(float x, float dx_dt)
{
    return x + dx_dt * drone.dt;
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