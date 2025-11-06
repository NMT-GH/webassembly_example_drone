#include "droneEstimation.h"
#include "math.h"
#include "kalman.h"
#include "linalg.h"
#include <stdio.h>
#include "droneSensors.h"


void attitudeComplementaryFilter(DRONE_T* drone)
{

    float biasToGyro = 0.99;

    //gyro part
    float gyroAngleEstimate;
    gyroAngleEstimate = drone->estimation.angle + drone->sensors.gyroscope * drone->dt;

    //accelerometer part
    float accelerometerAngleEstimate;
    accelerometerAngleEstimate = atan2f(drone->sensors.accelerometer.x, drone->sensors.accelerometer.y);

    //combine both parts
    drone->estimation.angle = biasToGyro * gyroAngleEstimate + (1-biasToGyro) * accelerometerAngleEstimate;
}

void pos_vel_estimate(DRONE_T* drone, int flag)
{

    MATRIX_T uInput; uInput.rows = 2; uInput.cols = 1;
    matSet(&uInput, drone->sensors.accelerometer.x, 0, 0);
    matSet(&uInput, drone->sensors.accelerometer.y, 1, 0);
    kalman_u_InputStep(&uInput, drone->estimation.angle);


    if(flag)
    {
        MATRIX_T zInput; zInput.rows = 4; zInput.cols = 1;
        matSet(&zInput, drone->sensors.GNSS_pos.x, 0, 0);
        matSet(&zInput, drone->sensors.GNSS_pos.y, 1, 0);
        matSet(&zInput, drone->sensors.GNSS_vel.x, 2, 0);
        matSet(&zInput, drone->sensors.GNSS_vel.y, 3, 0);

        kalman_z_InputStep(&zInput);
        kalmanStep();
    }
    else
    {
        kalmanStep_predictionOnly();
    }



    MATRIX_T state = kalmanGetState();
    drone->estimation.pos.x = state.arr[0];
    drone->estimation.pos.y = state.arr[1];
    drone->estimation.vel.x = state.arr[2];
    drone->estimation.vel.y = state.arr[3];

}