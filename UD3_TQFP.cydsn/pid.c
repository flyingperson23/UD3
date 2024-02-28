#include "pid.h"
#include <stdlib.h>
#include <stdint.h>

void constrain(int *x, int min, int max){
    if (*x > max) { *x = max; }
    if (*x < min) { *x = min; }
}

int PID_Update(PIDStruct *controller, int error_in) {
   
    // error
	//controller->E = error_in;
    controller->E = error_in * controller->Kp;
    constrain(&(controller->E), -02000000, 02000000);
    
    // integral
    if (controller->Ki != 0) {
        controller->I += controller->E;
    } else {
        controller->I = 0;
    }
    
    // derivative
    controller->D = controller->E - controller->E2;
    controller->E2 = controller->E;
    
    //controller->Y = (controller->Kp * controller->E * (error_in < 0 ? 1.6 : 1)) + ((controller->Ki * controller->I) / controller->Id) + (controller->Kd * controller->D);
    controller->Y = (controller->E * (error_in < 0 ? 1.6 : 1)) + ((controller->Ki * controller->I) / controller->Id) + (controller->Kd * controller->D);
    //constrain(&(controller->Y), -02000000, 02000000);
    return controller->Y;
}