#ifndef PID_H
#define PID_H

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

	} PIDStruct;


int PID_Update(PIDStruct *controller, int error_in);

void constrain(int *x, int min, int max);

#endif
