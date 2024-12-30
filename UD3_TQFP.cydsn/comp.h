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

#ifndef COMP_H
#define COMP_H

#include <stdint.h>

typedef struct{    
    int64_t Kp;
    int64_t Ki;
    int64_t Kd;
    int8_t kshift;
    int64_t a;
    int8_t ashift;
    int64_t E[2];
    int64_t d_P;
    int64_t d_I;
    int64_t d_D[2];
	} StructComp;


int64_t CompUpdate(StructComp *controller, int64_t error_in);
void CompInit(StructComp *controller, int64_t Kp, int64_t Ki, int64_t Kd, int64_t a, int8_t ashift, int8_t kshift);
void CompClear(StructComp *controller);

void constrain(int64_t *x, int64_t min, int64_t max);

#endif


/* [] END OF FILE */
