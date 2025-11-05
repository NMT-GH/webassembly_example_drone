#include "kalman.h"
#include <math.h>

static MATRIX_T P_pred;  //state covariance prediction
static MATRIX_T P_update; // state covariance update
static MATRIX_T Q; // covariance process noise
static MATRIX_T R; // covariance sensor noise
static MATRIX_T H; // sensor mapping
static MATRIX_T x_pred; //state prediction
static MATRIX_T x_update; //state update
static MATRIX_T y; //inovation
static MATRIX_T F; //state transition
static MATRIX_T B; //input effect
static MATRIX_T u; //input
static MATRIX_T z; //sensor
static MATRIX_T S; //innovation covariance
static MATRIX_T K; //kalman gain
static MATRIX_T I; //identity matrix
static MATRIX_T rotation_wb; 






//states for now: x, y, vx, vy, g
void setupKalman(float dt)
{
    //setup F
    F = matEye(5);
    matSet(&F, dt, 0, 2);
    matSet(&F, dt, 1, 3);
    matSet(&F, dt, 3, 4);
    matSet(&F, 0.5*powf(dt,2), 1, 4);

    //setup B
    B = matZeros(5, 2);
    matSet(&B, 0.5*powf(dt,2), 0, 0);
    matSet(&B, 0.5*powf(dt,2), 1, 1);
    matSet(&B, dt, 2, 0);
    matSet(&B, dt, 3, 1);

    //setup H
    H = matZeros(4,5);
    matSet(&H, 1, 0, 0);
    matSet(&H, 1, 1, 1);
    matSet(&H, 1, 2, 2);
    matSet(&H, 1, 3, 3);

    //setup P_update
    P_update = matZeros(5,5);
    matSet(&P_update, powf(0.005,  2), 0, 0);
    matSet(&P_update, powf(0.005,  2), 1, 1);
    matSet(&P_update, powf(0.0005, 2), 2, 2);
    matSet(&P_update, powf(0.0005, 2), 3, 3); 

    //setup Q
    Q = matZeros(5,5);
    matSet(&Q, powf(0.5*0.0014*powf(dt,2) + 0.001, 2), 0, 0);
    matSet(&Q, powf(0.5*0.0014*powf(dt,2) + 0.001, 2), 1, 1);
    matSet(&Q, powf(0.0014*dt             + 0.005, 2), 2, 2);
    matSet(&Q, powf(0.0014*dt             + 0.005, 2), 3, 3);

    //setup R
    R = matZeros(4,4);
    matSet(&R, pow(0.05 * 3.3, 2), 0, 0);
    matSet(&R, pow(0.05 * 5.3, 2), 1, 1);
    matSet(&R, pow(0.05 * 0.2, 2), 2, 2);
    matSet(&R, pow(0.05 * 0.2, 2), 3, 3);

    //setup x_update at t0
    x_update = matZeros(5,1);
    matSet(&x_update, -9.81, 4, 0);

    //setup I
    I = matEye(5);

    //setup u
    u.rows = 2; u.cols = 1;

    //setup z
    z.rows = 4; z.cols = 1;

    //setup rotation_wb
    rotation_wb.rows = 2;
    rotation_wb.cols = 2;
}

void kalman_u_InputStep(MATRIX_T* uInput, float angle)
{
    matSet(&rotation_wb,  cosf(angle), 0, 0);
    matSet(&rotation_wb, -sinf(angle), 0, 1);
    matSet(&rotation_wb,  sinf(angle), 1, 0);
    matSet(&rotation_wb,  cosf(angle), 1, 1);
    u = matMul(&rotation_wb, uInput);
}

void kalman_z_InputStep(MATRIX_T* zInput)
{
    z = *zInput;
}


void kalmanStep()
{
    MATRIX_T Fx = matMul(&F, &x_update);
    MATRIX_T Bu = matMul(&B, &u);
    x_pred = matAdd(&Fx, &Bu);

    MATRIX_T FP = matMul(&F, &P_update);
    MATRIX_T FT = matTranspose(&F);
    MATRIX_T FPFT = matMul(&FP, &FT);
    P_pred = matAdd(&FPFT, &Q);

    MATRIX_T Hx = matMul(&H, &x_pred);
    y = matSub(&z, &Hx);

    MATRIX_T HT = matTranspose(&H);
    MATRIX_T PHT = matMul(&P_pred, &HT);
    MATRIX_T HPHT = matMul(&H, &PHT);
    S = matAdd(&HPHT, &R);

    // A = PHT
    MATRIX_T AT = matTranspose(&PHT);
    MATRIX_T ST = matTranspose(&S);
    // ST * KT =  AT
    MATRIX_T KT = matSolve(&ST, &AT);
    K = matTranspose(&KT);

    MATRIX_T Ky = matMul(&K, &y);
    x_update = matAdd(&x_pred, &Ky);

    MATRIX_T KH = matMul(&K, &H);
    MATRIX_T IKH = matSub(&I, &KH);
    MATRIX_T IKHT = matTranspose(&IKH);
    MATRIX_T IKHP = matMul(&IKH, &P_pred);
    MATRIX_T IKHPIKHT = matMul(&IKHP, &IKHT);
    MATRIX_T KR = matMul(&K,&R);
    MATRIX_T KRKT = matMul(&KR, &KT);
    P_update = matAdd(&IKHPIKHT, &KRKT);

    matPrint(&x_update);
}


void kalmanStep_predictionOnly()
{
    MATRIX_T Fx = matMul(&F, &x_update);
    MATRIX_T Bu = matMul(&B, &u);
    x_pred = matAdd(&Fx, &Bu);
    x_update = x_pred;

    MATRIX_T FP = matMul(&F, &P_update);
    MATRIX_T FT = matTranspose(&F);
    MATRIX_T FPFT = matMul(&FP, &FT);
    P_pred = matAdd(&FPFT, &Q);
    P_update = P_pred;
}

MATRIX_T kalmanGetState()
{
    return x_update;
}