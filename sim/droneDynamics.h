#ifndef DRONE_DYNAMICS_H
#define DRONE_DYNAMICS_H

#include "drone.h"


void calc_accel(DRONE_T* , float , float );
void droneDynamicStep(DRONE_T* , float , float );


#endif