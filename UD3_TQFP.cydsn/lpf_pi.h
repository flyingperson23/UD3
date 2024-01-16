#ifndef LPF_PI_H
#define LPF_PI_H

typedef struct{    
    int Ki;  // integral constant
    int Kp;  // proportional constant
    int Kd;  // derivative constant
    int I;  // integral
    int E;  // error
    int E2; // last error
    int D;  // derivative
    int Y; // output
    int Id; // integral divisor

	} LPFStruct;


int LPF_Update(LPFStruct *controller, int error_in);

void constrain(int *x, int min, int max);

#endif
