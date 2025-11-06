#ifndef KALMAN_H
#define KALMAN_H

#include "linalg.h"

void setupKalman(float dt);
void kalman_u_InputStep(MATRIX_T* uInput, float angle);
void kalman_z_InputStep(MATRIX_T* zInput);
void kalmanStep();
void kalmanStep_predictionOnly();
MATRIX_T kalmanGetState();

#endif