/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "comp.h"
#include <stdlib.h>

void CompClear(StructComp *controller) {
    controller->E[0] = 0;
    controller->E[1] = 0;
    controller->d_P = 0;
    controller->d_I = 0;
    controller->d_D[0] = 0;
    controller->d_D[1] = 0;
}

void CompInit(StructComp *controller, int64_t Kp, int64_t Ki, int64_t Kd, int64_t a, int8_t ashift, int8_t kshift) {
    CompClear(controller);
    controller->Kp = Kp;
    controller->Ki = Ki;
    controller->Kd = Kd;
    controller->a = a;
    controller->ashift = ashift;
    controller->kshift = kshift;
}

int64_t CompUpdate(StructComp *controller, int64_t error_in) {
    controller->E[1] = controller->E[0];
    controller->d_D[1] = controller->d_D[0];
    controller->E[0] = error_in;
    
    controller->d_P = controller->Kp * error_in;
    controller->d_I += controller->Ki * error_in;
    controller->d_D[0] = (controller->E[0] + controller->E[1]) * controller->Kd
                       + ((controller->a * controller->d_D[1]) >> controller->ashift);
    
    return ((controller->d_P + controller->d_I + controller->d_D[0]) >> controller->kshift);
    
}

void constrain(int64_t *x, int64_t min, int64_t max){
    if (*x > max) { *x = max; }
    if (*x < min) { *x = min; }
}

/* [] END OF FILE */
