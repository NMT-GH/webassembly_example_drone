#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

#include "drone.h"

DRONE_EFFECTORS_T dronePositionController(VEC2D_T , DRONE_T* );
DRONE_EFFECTORS_T forceMomentController(float , float , DRONE_AIRFRAME_T* );
float angularVelocityController(float , float );
float attitudeController(float , float );
float targetWorldAccToTargetAtt(VEC2D_T );
float targetWorldAccToTargetAcc(VEC2D_T , float , float );
VEC2D_T targetWorldVelToTargetWorldAcc(VEC2D_T , VEC2D_T, DRONE_AIRFRAME_T*);
VEC2D_T targetPosToTargetVelocity(VEC2D_T , VEC2D_T,  DRONE_AIRFRAME_T* );
float constDecelWithSoftStopToVelocity(float , float , float );

#endif