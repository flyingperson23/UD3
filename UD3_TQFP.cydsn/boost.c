#include <device.h>
#include <stdint.h>
#include "boost.h"
#include "cli_common.h"
#include "tasks/tsk_fault.h"
#include "tasks/tsk_analog.h"
#include "alarmevent.h"
#include "telemetry.h"
#include "helper/teslaterm.h"
#include "helper/printf.h"
#include "helper/debug.h"
#include "ZCDtoPWM.h"

PIDStruct controller_V;
PIDStruct controller_I;
BoostStruct vars;


void stop() {
    temp_pwm_WriteCompare1(0);
    vars.dtc = 0;
    controller_I.I = 0;
    controller_V.I = 0;
    vars.i_target = 0;
}

CY_ISR(isr_boost) {
    // set vars
    if (tt.n.bus_v.divider > 0) {
        vars.v_bridge = tt.n.bus_v.value / tt.n.bus_v.divider;
    }
    vars.i_bridge = tt.n.batt_i.value;
    //check limits
    if (vars.v_target * tt.n.batt_v.divider > tt.n.batt_v.value * 1.2 && vars.v_bridge > 1.2 * vars.v_target) {
        if (sysfault.ov == 0) {
            alarm_push(ALM_PRIO_CRITICAL, "Bus: Overvoltage ", vars.v_bridge);
        }
        sysfault.ov = 1;
        stop();
    } else if (vars.i_bridge > configuration.max_fault_i) {
        if (sysfault.oc == 0) {
            alarm_push(ALM_PRIO_CRITICAL, "Bus: Overcurrent ", vars.i_bridge / 10);
        }
        sysfault.oc = 1;
        stop();
    } else if (bus_command != BUS_COMMAND_ON || tsk_fault_is_fault() || tt.n.batt_v.value * 1.2 > tt.n.batt_v.divider * vars.v_target) {
        stop();
    } else if (vars.v_target > 20) {
        //vars.i_target = PID_Update(&controller_V, vars.v_target - vars.v_bridge);
        vars.i_target = vars.i_target * 9 / 10 + vars.v_target - vars.v_bridge;
        constrain(&vars.i_target, 0, configuration.max_dc_curr);
        
        if (vars.i_target < 10) {
            stop();
        }
        
        vars.v_rq = vars.v_bridge + PID_Update(&controller_I, vars.i_target - vars.i_bridge) / 100;
        constrain(&vars.v_rq, 0, vars.v_target);
        
        if (vars.v_bridge > 50) {
            vars.dtc = vars.v_rq * getPeriod() / vars.v_bridge;
        } else {
            vars.dtc = vars.v_rq * getPeriod() / 10;
        }
        constrain(&vars.dtc, 0, getPeriod() * 8 / 10); // max 80% dtc
        
        // write dtc
        temp_pwm_WriteCompare1(vars.dtc);
    }
}

void boost_init() {
    boost_isr_StartEx(isr_boost);
    
    controller_V.Ki = 0;
    controller_V.Id = 1;
    controller_V.Kp = 1;
    controller_V.Kd = 0;
    
    
    controller_I.Ki = 25;
    controller_I.Id = 100;
    controller_I.Kp = 15;
    controller_I.Kd = 0;
}

uint16_t getPeriod() {
    return temp_pwm_ReadPeriod() + 1;
}