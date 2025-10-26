#include "drone.h"
#include "droneDynamics.h"
#include "droneController.h"
#include <math.h>

#define P_GAIN_ANGULAR_VELOCITY  40
#define P_GAIN_ANGLE  20
#define P_GAIN_WORLD_VEL_WORLD_ACC  10
#define P_GAIN_POS_VEL  3


DRONE_EFFECTORS_T dronePositionController(VEC2D_T targetPos, DRONE_T* drone)
{
    VEC2D_T targetVelocity     = targetPosToTargetVelocity(targetPos, drone->states.pos);
    VEC2D_T targetAcceleration = targetWorldVelToTargetWorldAcc(targetVelocity, drone->states.vel);
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
    return atanf( -(targetAcc.x / (targetAcc.y + GRAVITY)) );
}

float targetWorldAccToTargetAcc(VEC2D_T targetAcc, float targetAtt, float currentAtt)
{

    float totalAcc = sqrtf( powf(targetAcc.x,2) + powf(targetAcc.y + GRAVITY, 2) );

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

    if (targetAcceleration.y < -(GRAVITY * 0.9)){targetAcceleration.y = -(GRAVITY * 0.9);} 


    


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
