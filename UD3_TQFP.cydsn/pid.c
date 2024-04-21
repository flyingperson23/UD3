#include "pid.h"
#include <stdlib.h>
#include <stdint.h>

void constrain(int *x, int min, int max){
    if (*x > max) { *x = max; }
    if (*x < min) { *x = min; }
}

int PID_Update(PIDStruct *controller, int error_in) {
    controller->m[1] = controller->m[0];
    controller->e[2] = controller->e[1];
    controller->e[1] = controller->e[0];
    controller->e[0] = error_in;
    controller->m[0] = (controller->Kp + controller->Ki + controller->Kd) * controller->e[0] - (controller->Kp + 2*controller->Kd) * controller->e[1] + controller->Kd * controller->e[2] + controller->m[1];
    return controller->m[0];
}