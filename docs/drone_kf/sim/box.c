#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "drone.h"
#include "droneDynamics.h"
#include "droneController.h"
#include "droneSensors.h"
#include "droneEstimation.h"
#include "linalg.h"
#include "kalman.h"
#include <emscripten/emscripten.h>

DRONE_T drone;

// CONSTANTS ----------
float gravity = 9.81;
int counter = 0;
int gps_flag = 0;



uint8_t sim_init(float dt);
void    sim_step(float thr, float steer);
float   drone_get_x(void);
float   drone_get_y(void);
float   drone_get_angle(void);


uint8_t sim_init(float dt)
{
    srand(0);

    drone.dt = dt;
    drone.airframe.mass = 0.25; //250g
    drone.airframe.inertia = 5 * 1e-5;
    drone.airframe.propDist = 0.127/2; //127mm cg to motor center
    drone.airframe.maxThrust = 3; //3N per prop


    setupKalman(dt);


    // test matrix stuff
    // MATRIX_T X = matZeros(4,4);
    // matSet(&X, 20, 0, 0);
    // matSet(&X, -8, 0, 1);
    // matSet(&X, 30, 0, 2);
    // matSet(&X, -1, 0, 3);
    // matSet(&X, -1, 1, 0);
    // matSet(&X, -9, 1, 1);
    // matSet(&X, 10, 1, 2);
    // matSet(&X, -2, 1, 3);
    // matSet(&X, 40, 2, 0);
    // matSet(&X, -2, 2, 1);
    // matSet(&X, 50, 2, 2);
    // matSet(&X, -3, 2, 3);
    // matSet(&X, 40, 3, 0);
    // matSet(&X, -2, 3, 1);
    // matSet(&X, 50, 3, 2);
    // matSet(&X, -3, 3, 3);

    // QR_T qr = matQR(&X);

    // matPrint(&(qr.Q));
    // matPrint(&(qr.R));
    
    // MATRIX_T QRtest = matMul((&qr.Q), &(qr.R));
    // matPrint(&X);
    // matPrint(&QRtest);


    // solve test -----
    // MATRIX_T A; A.cols = 3; A.rows = 3;
    // MATRIX_T B; B.cols = 2; B.rows = 3;

    // matSet(&A, -2, 0, 0);
    // matSet(&A,  8, 0, 1);
    // matSet(&A,  1, 0, 2);

    // matSet(&A, -2, 1, 0);
    // matSet(&A,  1, 1, 1);
    // matSet(&A,  0, 1, 2);

    // matSet(&A,  9, 2, 0);
    // matSet(&A, -3, 2, 1);
    // matSet(&A, -5, 2, 2);

    // matSet(&B,  2, 0, 0);
    // matSet(&B,  1, 0, 1);

    // matSet(&B, -7, 1, 0);
    // matSet(&B,  6, 1, 1);

    // matSet(&B,  1, 2, 0);
    // matSet(&B, 15, 2, 1);

    // MATRIX_T X = matSolve(&A, &B);
    // matPrint(&X);






    return 0;
}

void sim_step(float targetPos_x, float targetPos_y)
{

    counter++;

    VEC2D_T targetPos;
    targetPos.x = targetPos_x;
    targetPos.y = targetPos_y;


    DRONE_EFFECTORS_T effector = dronePositionController(targetPos, &drone);

    droneDynamicStep(&drone, effector.left, effector.right);
    
    accelerometerMeasurement(&drone);
    gyroscopeMeasurement(&drone);



    attitudeComplementaryFilter(&drone);

    if(counter>10)
    {
        counter = 0;

        GNSSMeasurement_position(&drone);
        GNSSMeasurement_velocity(&drone);

        pos_vel_estimate(&drone, 1);
    }
    else
    {
        pos_vel_estimate(&drone, 0);
    }

    
    

    

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

float drone_get_x_estimate()
{
    return drone.estimation.pos.x;
}

float drone_get_y_estimate()
{
    return drone.estimation.pos.y;
}

EMSCRIPTEN_KEEPALIVE
float drone_get_angle_estimate()
{
    return drone.estimation.angle;
}

EMSCRIPTEN_KEEPALIVE
float drone_get_gnss_x()
{
    return drone.sensors.GNSS_pos.x;
}

EMSCRIPTEN_KEEPALIVE
float drone_get_gnss_y()
{
    return drone.sensors.GNSS_pos.y;
}