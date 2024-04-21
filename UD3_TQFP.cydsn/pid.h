#ifndef PID_H
#define PID_H

typedef struct{    
    int Ki;  // integral constant
    int Kp;  // proportional constant
    int Kd;  // derivative constant
    int m[5]; // output
    int e[5]; // error

	} PIDStruct;


int PID_Update(PIDStruct *controller, int error_in);

void constrain(int *x, int min, int max);

#endif
