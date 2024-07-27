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

/* [] END OF FILE */

#ifndef THERM_H
#define THERM_H

#include <stdint.h>

uint8_t therm_convert(uint16_t adc);
void therm_init();
extern uint8_t thermlut[256];
#endif
