#ifndef DRONE_ESTIMATION_H
#define DRONE_ESTIMATION_H

#include "drone.h"

void attitudeComplementaryFilter(DRONE_T* );
void pos_vel_estimate(DRONE_T* , int);

#endif