#ifndef LINALG_H
#define LINALG_H

typedef struct{
    // Matrix [10 20
    //         30 40]
    // is stored as [10 20 30 40]
    float arr[32]; // wasteful and doesnt work for larger matrices
    char cols;
    char rows;
} MATRIX_T;

typedef struct{
    MATRIX_T Q;
    MATRIX_T R;
} QR_T;


MATRIX_T matAdd(MATRIX_T* , MATRIX_T* ) ;
MATRIX_T matSub(MATRIX_T* , MATRIX_T* ) ;
MATRIX_T matMul(MATRIX_T* , MATRIX_T* ) ;
MATRIX_T matTranspose(MATRIX_T* );
MATRIX_T matSolve(MATRIX_T*, MATRIX_T*);
float matScalarProduct(MATRIX_T* );
MATRIX_T matOuterProduct(MATRIX_T* );
MATRIX_T matTimesScalar(MATRIX_T* ,  float );
MATRIX_T matEye(int );
MATRIX_T matZeros(int , int );
MATRIX_T matBasisVector(float , int );
void matPrint(MATRIX_T*);
MATRIX_T matExtractCol(MATRIX_T* , int );
float matVecMag(MATRIX_T* );
QR_T matQR(MATRIX_T* );
MATRIX_T matRBS(MATRIX_T* , MATRIX_T* );


// inline functions
static inline void matSet(MATRIX_T* A, float value, int row, int col)
{
    A->arr[row * A->cols + col] = value;
} 
static inline float matGet(MATRIX_T* A, int row, int col)
{
    return A->arr[row * A->cols + col];
} 


#endif