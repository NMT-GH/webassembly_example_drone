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


#endif