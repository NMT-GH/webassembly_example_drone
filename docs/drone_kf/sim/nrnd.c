#include "nrnd.h"
#include "stdlib.h"
#include "math.h"


float nrnd(float mean, float stddev) {
    // Use Box-Muller transform
    float u1 = ((float) rand() + 1) / ((float) RAND_MAX + 2);
    float u2 = ((float) rand() + 1) / ((float) RAND_MAX + 2);

    float z0 = sqrtf(-2.0 * logf(u1)) * cosf(2.0 * PI * u2);
    return z0 * stddev + mean;
}