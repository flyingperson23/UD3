
#include <device.h>
#include "boost.h"
#include "cli_common.h"
#include "tasks/tsk_fault.h"
#include "tasks/tsk_analog.h"
#include "alarmevent.h"

void stop() {
    //vars.v_target = 0;
    //vars.i_target = 0;
    setIMax();
    temp_pwm_WriteCompare1(0);
    vars.dtc = 0;
}

CY_ISR(isr_boost) {
    //check limits
    if (vars.v_bridge / vars.v_target > 1.2f) {
        if (sysfault.ov == 0) {
            alarm_push(ALM_PRIO_CRITICAL, "Bus: Overvoltage ", vars.v_bridge);
        }
        sysfault.ov = 1;
        stop();
    } else if (vars.i_bridge / vars.i_target > 1.2f) {
        if (sysfault.oc == 0) {
            alarm_push(ALM_PRIO_CRITICAL, "Bus: Overcurrent ", vars.i_bridge);
        }
        sysfault.oc = 1;
        stop();
    } else if (bus_command != BUS_COMMAND_ON || tsk_fault_is_fault()) {
        stop();
    } else {
        // set vars
    
        // get dtc's
        vars.dtc_v = LPF_Update(&controller_V, vars.v_target - vars.v_bridge);
        vars.dtc_i = LPF_Update(&controller_I, vars.i_target - vars.i_bridge);
        
        // min
        if (vars.dtc_v > vars.dtc_i) {
            vars.dtc += vars.dtc_i;
        } else {
            vars.dtc += vars.dtc_v;
        }
        
        // write dtc
        temp_pwm_WriteCompare1(temp_pwm_ReadPeriod() * vars.dtc);
    }
}

void setIMax() {
    vars.i_target = (float) configuration.max_dc_curr / 10.0f;
}

void boost_init() {
    boost_isr_StartEx(isr_boost);
    
    controller_I.lag_ki = 0.05f;
    controller_I.Kp = 2.0f;
    controller_I.lpf_pole = 0.6f;
    controller_I.cmd_lim_min = 0.0f;
    controller_I.cmd_lim_max = 0.8f;
    
    controller_V.lag_ki = 0.05f;
    controller_V.Kp = 2.0f;
    controller_V.lpf_pole = 0.6f;
    controller_V.cmd_lim_min = 0.0f;
    controller_V.cmd_lim_max = 0.8f;
    
    setIMax();
}
