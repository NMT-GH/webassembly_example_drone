#include "linalg.h"
#include "string.h"
#include <stdio.h>
#include <math.h>




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

// A^T
MATRIX_T matTranspose(MATRIX_T* A)
{
    MATRIX_T AT;
    AT.cols = A->rows;
    AT.rows = A->cols;

    for(int A_row = 0; A_row < A->rows; A_row++)
    {
        for(int A_col = 0; A_col < A->cols; A_col++)
        {
            matSet(&AT, matGet(A, A_row, A_col), A_col, A_row);
        }
    }

    return AT;
}

// vector a^T * a
float matScalarProduct(MATRIX_T* a)
{
    MATRIX_T aT;
    aT = matTranspose(a);

    MATRIX_T aTa;
    aTa = matMul(&aT, a);

    return aTa.arr[0];
}

MATRIX_T matOuterProduct(MATRIX_T* a)
{
    MATRIX_T aT;
    aT = matTranspose(a);

    MATRIX_T aaT;
    aaT = matMul(&aT, a);

    return aaT;
}

MATRIX_T matTimesScalar(MATRIX_T* A, float s)
{
    MATRIX_T X;
    X.rows = A->rows;
    X.cols = A->cols;

    for(int iter = 0; iter < (A->cols * A->rows); iter++)
    {
        X.arr[iter] = A->arr[iter] * s;
    }

    return X;
}

MATRIX_T matEye(int numdim)
{
    MATRIX_T X;

    X.rows = numdim;
    X.cols = numdim;

    for(int iterRow = 0; iterRow < numdim; iterRow++)
    {
        for(int iterCol = 0; iterCol < numdim; iterCol++)
        {
            if(iterCol == iterRow)
            {
                matSet(&X, 1, iterRow, iterCol);
            }
            else
            {
                matSet(&X, 0, iterRow, iterCol);
            }
        }
    }
    return X;
}

MATRIX_T matZeros(int numRows, int numCols)
{
    MATRIX_T X;

    X.rows = numRows;
    X.cols = numCols;

    for(int iter = 0; iter < numRows*numCols; iter++)
    {
        X.arr[iter] = 0;
    }

    return X;
}

// place is zero based, so matBasisVector(3,1) = [0 1 0]'
MATRIX_T matBasisVector(float numRows, int place)
{
    MATRIX_T X;

    X.rows = numRows;
    X.cols = 1;

    for(int iter = 0; iter < numRows; iter++)
    {
        X.arr[iter] = 0;
    }
    X.arr[place] = 1;

    return X;
}

// H = I - 2 v v^T
MATRIX_T matHouseholder(MATRIX_T* I, MATRIX_T* u)
{
    
    MATRIX_T u2 = matTimesScalar(u, 2);
    MATRIX_T uT = matTranspose(u);
    float uTu = matScalarProduct(u);

    MATRIX_T u2uT = matMul(&u2, &uT);
    MATRIX_T u2uT_D_uTu = matTimesScalar(&u2uT, 1/uTu);


    return matSub(I, &u2uT_D_uTu);
}

MATRIX_T matExtractCol(MATRIX_T* A, int selectedColumn)
{
    MATRIX_T X;
    
    X.cols = 1;
    X.rows = A->rows;


    for(int row = 0; row < X.rows; row++)
    {
        X.arr[row] = A->arr[row * A->cols + selectedColumn];
    }

    return X;
}

float matVecMag(MATRIX_T* v)
{
    float sum = 0;
    
    for(int iter = 0; iter < v->rows; iter++)
    {
        sum += v->arr[iter] * v->arr[iter];
    }

    return sqrtf(sum);
}

QR_T matQR(MATRIX_T* X)
{   
    QR_T qr;

    qr.Q = matEye(X->rows);
    qr.R = *X;

    for(int iter = 0; iter < X->rows-1; iter++)
    {
        MATRIX_T eb = matBasisVector(qr.R.rows, iter);
        MATRIX_T xc = matExtractCol(&(qr.R),iter);

        for(int zeroIter = 0; zeroIter < iter; zeroIter++)
        {
            matSet(&xc, 0, zeroIter, 0);
        }

        float xcMag = matVecMag(&xc);
        
        MATRIX_T ebs = matTimesScalar(&eb, xcMag);
        
        MATRIX_T I = matEye(qr.R.rows);
        MATRIX_T u = matSub(&xc, &ebs);

        MATRIX_T H = matHouseholder(&I, &u);
        qr.Q = matMul(&H, &(qr.Q));
        qr.R = matMul(&H, &(qr.R));
    }

    qr.Q = matTranspose(&(qr.Q));

    return qr;
}

//A * X = B
MATRIX_T matSolve(MATRIX_T* A, MATRIX_T* B)
{

    QR_T A_QR = matQR(A);

    MATRIX_T QT  = matTranspose(&(A_QR.Q));
    MATRIX_T QTB = matMul(&QT, B);

    return matRBS(&(A_QR.R), &QTB);

}

// upper triangular backsubstitution R*X = B
MATRIX_T matRBS(MATRIX_T* R, MATRIX_T* B)
{
    float den = 0;
    float num = 0;

    MATRIX_T X;
    X.rows = R->cols;
    X.cols = B->cols;


    for(int colB = 0; colB < B->cols; colB++)
    {
        for(int rowR = R->rows-1; rowR >= 0; rowR--)
        {

            num = matGet(B, rowR, colB);

            for(int iter = rowR + 1; iter < R->rows; iter++)
            {
                num -= matGet(R, rowR, iter) * matGet(&X, iter, colB);
            }
            den = matGet(R, rowR, rowR);

            matSet(&X, num/den, rowR, colB);
        }
    }

    return X;
}



void matPrint(MATRIX_T* A)
{
    
    printf("Rows: %d\n", A->rows);
    printf("Columns: %d\n", A->cols);
    
    
    for(int row = 0; row < A->rows; row++)
    {
        for(int col = 0; col < A->cols; col++)
        {
            printf("%.2f ", matGet(A, row, col));
        }

        printf("\n");
    }

    printf("\n\n\n");
}