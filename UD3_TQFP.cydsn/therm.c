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

#include "therm.h"
#include <math.h>
#include "cli_common.h"

uint8_t thermlut[256];

void therm_init() {
    for (int i = 0; i < 256; i++) {
        thermlut[i] = therm_convert((i << 4) + 1);
    }
}

uint8_t therm_convert(uint16_t adc) {
    if (adc != 4096) {
        double voltage = adc * 5.0 / 4096.0;
        double R = 10000.0*voltage/(5.0 - voltage);
        double a = log(R/((double) configuration.ntc_r25)) / ((double) configuration.ntc_b) + (1.0 / 298.15);
        if (a != 0) {
            double c = (1.0 / a) - 273.15;
            if (c < 0) c = 0;
            if (c > 255) c = 255;
            return (uint8_t) c;
        }
    }
    return 0;
}