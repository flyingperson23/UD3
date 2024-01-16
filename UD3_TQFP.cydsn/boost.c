
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

LPFStruct controller_V;
LPFStruct controller_I;
BoostStruct vars;


void stop() {
    //vars.v_target = 0;
    //vars.i_target = 0;
    setIMax();
    temp_pwm_WriteCompare1(0);
    vars.dtc = 0;
}
int temp;
CY_ISR(isr_boost) {
    // set vars
    if (tt.n.bus_v.divider > 0) {
        vars.v_bridge = tt.n.bus_v.value / tt.n.bus_v.divider;
    }
    if (tt.n.batt_i.divider > 0) {
        vars.i_bridge = tt.n.batt_i.value / tt.n.batt_i.divider;
    }
    //check limits
    //alarm_push(ALM_PRIO_INFO, "boostin'", ALM_NO_VALUE);
    if (vars.v_target * tt.n.batt_v.divider > tt.n.batt_v.value * 1.2 && vars.v_bridge > 1.2 * vars.v_target) {
        if (sysfault.ov == 0) {
            alarm_push(ALM_PRIO_CRITICAL, "Bus: Overvoltage ", vars.v_bridge);
        }
        sysfault.ov = 1;
        stop();
    } else if (vars.i_bridge * 10 > configuration.max_fault_i) {
        if (sysfault.oc == 0) {
            alarm_push(ALM_PRIO_CRITICAL, "Bus: Overcurrent ", vars.i_bridge);
        }
        sysfault.oc = 1;
        stop();
    } else if (bus_command != BUS_COMMAND_ON || tsk_fault_is_fault() || tt.n.batt_v.value * 1.2 > tt.n.batt_v.divider * vars.v_target) {
        stop();
    } else if (vars.v_target > 20) {
        
        // get dtc's
        vars.dtc_v = LPF_Update(&controller_V, vars.v_target - vars.v_bridge);
        //vars.dtc_i = LPF_Update(&controller_I, vars.i_target - vars.i_bridge);
        
        //vars.dtc_v = (vars.v_target - vars.v_bridge) * controller_V.Kp;
        //vars.dtc_i = (vars.i_target - vars.i_bridge) * controller_I.Kp;
        
        // min
        //if (vars.dtc_v > vars.dtc_i) {
        //  vars.dtc += vars.dtc_i;
        //} else {
        //    vars.dtc += vars.dtc_v;
        //}
        //constrain(&(vars.dtc), 0, 0.8f);
        vars.dtc += vars.dtc_v;
        
        if (vars.dtc > 80000000) { // 80%
            vars.dtc = 80000000;
        }
        if (vars.dtc < 0) {
            vars.dtc = 0;
        }
        
        temp = vars.dtc;
        temp /= 10000;
        temp *= 2720;
        temp /= 10000;
        
        
        // write dtc
        //temp_pwm_WriteCompare1(temp_pwm_ReadPeriod() * vars.dtc);
        temp_pwm_WriteCompare1(temp);
    }
}

void setIMax() {
    vars.i_target = configuration.max_dc_curr / 10;
}

void boost_init() {
    boost_isr_StartEx(isr_boost);
    
    //controller_I.lag_ki = 0.0004f;
    //controller_I.Kp = 0.0004f;
    
    controller_V.Ki = 0;
    controller_V.Kp = 50;
    controller_V.Kd = 85000;
    controller_V.Id = 10000;
    
    setIMax();
}
