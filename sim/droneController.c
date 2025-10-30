#include "drone.h"
#include "droneDynamics.h"
#include "droneController.h"
#include <math.h>
#include <stdio.h>

#define P_GAIN_ANGULAR_VELOCITY  40
#define P_GAIN_ANGLE  30
#define P_GAIN_WORLD_VEL_WORLD_ACC  11
#define P_GAIN_POS_VEL  4.5


DRONE_EFFECTORS_T dronePositionController(VEC2D_T targetPos, DRONE_T* drone)
{
    VEC2D_T targetVelocity     = targetPosToTargetVelocity(targetPos, drone->states.pos, &(drone->airframe));
    VEC2D_T targetAcceleration = targetWorldVelToTargetWorldAcc(targetVelocity, drone->states.vel, &(drone->airframe));
    float targetAttitude       = targetWorldAccToTargetAtt(targetAcceleration);
    float targetTotalAcc       = targetWorldAccToTargetAcc(targetAcceleration, targetAttitude, drone->estimation.angle);

    float targetAngularVel     = attitudeController(targetAttitude, drone->estimation.angle);
    float targetAngularAcc     = angularVelocityController(targetAngularVel, drone->sensors.gyroscope);

    DRONE_EFFECTORS_T effector = forceMomentController(targetTotalAcc, targetAngularAcc, &(drone->airframe));
    return effector;
}

DRONE_EFFECTORS_T forceMomentController(float targetAccel, float targetAngularAccel, DRONE_AIRFRAME_T* airframe)
{

    DRONE_EFFECTORS_T effector; 

    // summedThrustInput * maxThrust / droneMass = targetAccel
    float summedThrustInput = targetAccel * airframe->mass / airframe->maxThrust;
    
    // differenceThrustOutput * maxThrust * propdist / intertia = angualr_acc
    float differenceThrustOutput = targetAngularAccel * airframe->inertia / (airframe->maxThrust * airframe->propDist);

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
    return atan2f( -targetAcc.x, (targetAcc.y + GRAVITY) );
}

float targetWorldAccToTargetAcc(VEC2D_T targetAcc, float targetAtt, float currentAtt)
{

    float totalAcc = sqrtf( powf(targetAcc.x,2) + powf(targetAcc.y + GRAVITY, 2) );

    float alignmentFactor = powf(cosf(targetAtt - currentAtt),1);
    if (((targetAtt - currentAtt) > (3.14/2)) || ((targetAtt - currentAtt) < (-3.14/2)))
    {
        alignmentFactor = 0;
    }
    

    return totalAcc * alignmentFactor;
}

VEC2D_T targetWorldVelToTargetWorldAcc(VEC2D_T targetVel, VEC2D_T currentVel,  DRONE_AIRFRAME_T* airframe)
{
    VEC2D_T errorVelocity;
    VEC2D_T targetAcceleration;

    errorVelocity.x = targetVel.x - currentVel.x;
    errorVelocity.y = targetVel.y - currentVel.y;

    targetAcceleration.x = errorVelocity.x * P_GAIN_WORLD_VEL_WORLD_ACC;
    targetAcceleration.y = errorVelocity.y * P_GAIN_WORLD_VEL_WORLD_ACC;


    float acc_angle = atan2f(-targetAcceleration.x, targetAcceleration.y);
    float acc_magni = sqrtf(powf(targetAcceleration.x,2) + powf(targetAcceleration.y,2));

    float acc_max_magni = -GRAVITY * cosf(acc_angle) + sqrtf(powf(2*(airframe->maxThrust/airframe->mass), 2) - powf(GRAVITY * sinf(acc_angle), 2));

    if (acc_magni > 0.9 * acc_max_magni){
        targetAcceleration.x = -sinf(acc_angle) * 0.9 * acc_max_magni;
        targetAcceleration.y =  cosf(acc_angle) * 0.9 * acc_max_magni;

        // printf("targetAcceleration.x: %.4f\r\n", targetAcceleration.x);
        // printf("targetAcceleration.y: %.4f\r\n", targetAcceleration.y);
        // printf("acc_max_magni: %.4f\r\n", acc_max_magni);
        // printf("acc_magni: %.4f\r\n", acc_magni);
        // printf("acc_angle: %.4f\r\n", acc_angle);
    }



    //if (targetAcceleration.y < -(GRAVITY * 0.9)){targetAcceleration.y = -(GRAVITY * 0.9);} 


    //printf("targetAcceleration.y: %.4f\r\n", targetAcceleration.y);


    return targetAcceleration;
}

VEC2D_T targetPosToTargetVelocity(VEC2D_T targetPos, VEC2D_T currentPos, DRONE_AIRFRAME_T* airframe)
{
    VEC2D_T errorPosition;
    VEC2D_T targetVelocity;

    errorPosition.x = targetPos.x - currentPos.x;
    errorPosition.y = targetPos.y - currentPos.y;

    // float angle     = atan2f(errorPosition.y, errorPosition.x);
    // float magnitude = sqrtf(powf(errorPosition.x,2) + powf(errorPosition.y,2));

    float pos_angle = atan2f(errorPosition.y, errorPosition.x);
    float neg_acc_angle = atan2f(errorPosition.x, -errorPosition.y);
    float pos_magni = sqrtf(powf(errorPosition.x,2) + powf(errorPosition.y,2));

    float acc_max_magni = -GRAVITY * cosf(neg_acc_angle) + sqrtf(powf(2*(airframe->maxThrust/airframe->mass), 2) - powf(GRAVITY * sinf(neg_acc_angle), 2));

    float targetVelocityMagnitude = constDecelWithSoftStopToVelocity(pos_magni, 0.15*acc_max_magni, P_GAIN_POS_VEL);


    targetVelocity.x = cos(pos_angle) * targetVelocityMagnitude;
    targetVelocity.y = sin(pos_angle) * targetVelocityMagnitude;


    // targetVelocity.x = errorPosition.x * P_GAIN_POS_VEL;
    // targetVelocity.y = errorPosition.y * P_GAIN_POS_VEL;

    //printf("targetVelocity.y: %.4f\r\n", targetVelocity.y);

    return targetVelocity;
}


float constDecelWithSoftStopToVelocity(float x, float a, float c)
{

    float h = (a/powf(c,2));
    float velocity;

    if(x < h)
    {
        // Within soft stop regime
        // proportional feedback for good 
        velocity = c*x;
    } 
    else
    {
        // Outside of softstop regime
        // accel/decel with a 
        velocity = sqrtf( 2*a * (x - 0.5 * h) );
    }

    return velocity;
}
