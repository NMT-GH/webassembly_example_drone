#include "linalg.h"
#include "string.h"

// X = A + B
MATRIX_T matAdd(MATRIX_T* A, MATRIX_T* B) 
{

    MATRIX_T X;

    // check correct dimension
    if(!((A->rows == B->rows) && (A->cols == B->cols)))
    {
         // mismatch in dimentions
    }

    X.cols = A->cols;
    X.rows = A->rows;

    for(int iter = 0; iter < (A->rows*A->cols); iter++ )
    {
        X.arr[iter] = A->arr[iter] + B->arr[iter];
    }

    return X;
}

// X = A - B
MATRIX_T matSub(MATRIX_T* A, MATRIX_T* B) 
{

    MATRIX_T X;
    // check correct dimension
    if(!((A->rows == B->rows) && (A->cols == B->cols)))
    {
         // mismatch in dimentions
    }

    X.cols = A->cols;
    X.rows = A->rows;


    for(int iter = 0; iter < (A->rows*A->cols); iter++ )
    {
        X.arr[iter] = A->arr[iter] - B->arr[iter];
    }

    return X;
}


// X = A * B
MATRIX_T matMul(MATRIX_T* A, MATRIX_T* B) 
{
    MATRIX_T X;

    // check correct dimension
    if(!((A->cols == B->rows)))
    {
         // mismatch in dimentions
    }

    X.rows = A->rows;
    X.cols = B->cols;

    //X->arr[0] = A->arr[0]*B->arr[0] + A->arr[1]*B->arr[B->cols] + A->arr[2]*B->arr[B->cols * 2]

    memset(X.arr, 0, 4* A->rows * B->cols);

    for(int rightIter = 0; rightIter < B->cols; rightIter++)
    {
        for(int downIter = 0; downIter < A->rows; downIter++)
        {
            for(int iter = 0; iter < A->cols; iter++)
            {
                X.arr[ X.cols * downIter + rightIter] += A->arr[iter + downIter * A->cols] * B->arr[iter * B->cols + rightIter];
            }
        }
    }

    return X;
}

