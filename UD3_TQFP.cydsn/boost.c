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

StructComp controller_V;
StructComp controller_I;
BoostStruct vars;

uint32_t bus_voltage_factor = 0;
int64_t dtc_constraint = 0;

uint32_t read_bus_mv22(uint16_t raw_adc) {
	uint32_t bus_voltage;
	bus_voltage = ((configuration.r_top + BUSV_R_BOT) * raw_adc) / (BUSV_R_BOT * 819 / 1000);
	return bus_voltage;
}


void stop() {
    temp_pwm_WriteCompare1(0);
    temp_pwm_WriteCompare2(0);
    vars.dtc = 0;
    vars.i_target = 0;
    vars.v_target2 = 0;
    
    CompClear(&controller_V);
    CompClear(&controller_I);
}

CY_ISR(isr_ibus) {
    int64_t ibus_reading = adc_dma_array[0] + 7; // offset of 7?
    if (ibus_reading < 0) {
        ibus_reading = 0;
    }
    vars.i_bridge = ibus_reading * params.idc_ma_count; // current in ma    
    //boost_loop();
}

CY_ISR(isr_boost) {
    boost_loop();
}

int counter = 0;
int vloop_cnt = 0;
void boost_loop() {
    //check limits
    if (vars.v_target * tt.n.batt_v.divider * 10 > tt.n.batt_v.value * 12 * 1000 && vars.v_bridge * 10 > 12 * vars.v_target) {
        if (sysfault.ov == 0) {
            alarm_push(ALM_PRIO_CRITICAL, "Bus: Overvoltage ", vars.v_bridge / 1000);
        }
        sysfault.ov = 1;
        stop();
    } else if (tt.n.batt_i.value > configuration.max_fault_i) {
        if (sysfault.oc == 0) {
            alarm_push(ALM_PRIO_CRITICAL, "Bus: Overcurrent ", tt.n.batt_i.value / 10);
        }
        sysfault.oc = 1;
        stop();
    } else if (bus_command != BUS_COMMAND_ON || tsk_fault_is_fault()) {
        stop();
    } else if (vars.v_target > 20000) {
        if (vars.v_target2 < vars.v_in) {
            vars.v_target2 = vars.v_in;
        }
        if (vars.v_target2 > vars.v_target) {
            vars.v_target2-= 1000;
        } else if (vars.v_target2 < vars.v_target) {
            counter++;
            if (counter % 8 == 0) {
                vars.v_target2+= 1000;
            }
        }
        constrain(&vars.v_target2, 0, vars.v_bridge + 300000);
        vars.dtc = CompUpdate(&controller_I, vars.i_target - vars.i_bridge);
        // takes in ma off, outputs dtc as [0, 1024)
        if (vars.v_bridge + 100 >= vars.v_target2) vars.dtc = 0;
        
        constrain(&vars.dtc, 0, dtc_constraint);
        
        // write dtc
        temp_pwm_WriteCompare1(vars.dtc);
        temp_pwm_WriteCompare2(vars.dtc >> 1); // half duty cycle for adc trig
        
        vloop_cnt++;
        
        if(vloop_cnt >= 10){
        
            // set vars
            //for (int i = 0; i < 15; i++) {
            //    vars.v_bridge += read_bus_mv22(ADC_active_sample_buf[i].v_bus);
            //    vars.v_in += read_bus_mv22(ADC_active_sample_buf[i].v_batt); // in mv
            //}
            //vars.v_bridge = vars.v_bridge >> 4;
            //vars.v_in = vars.v_in >> 4;

           
            
                
            vloop_cnt = 0;
            vars.i_target = CompUpdate(&controller_V, vars.v_target2 - vars.v_bridge) * vars.v_in / (tt.n.batt_v.value * tt.n.batt_v.value);
            vars.i_target *= 1000; // convert A to mA
            constrain(&vars.i_target, -configuration.max_dc_curr * 100, configuration.max_dc_curr * 100);
            if (vars.v_bridge + 10000 >= vars.v_target2) vars.i_target = 0;
            if (vars.i_target > 0 && vars.i_target < 100) vars.i_target = 0;
            if (vars.i_target < 0 && vars.i_target > -100) vars.i_target = 0;
        }
        
    } else {
        stop();
    }
}

void dma_config() {    
    uint8 DMA_Ibus_Chan;
    uint8 DMA_Ibus_TD[1];

	/* DMA Configuration for DMA_Ibus */
	#define DMA_Ibus_BYTES_PER_BURST 2
	#define DMA_Ibus_REQUEST_PER_BURST 1
	#define DMA_Ibus_SRC_BASE (CYDEV_PERIPH_BASE)
	#define DMA_Ibus_DST_BASE (CYDEV_SRAM_BASE)
	DMA_Ibus_Chan = DMA_Ibus_DmaInitialize(DMA_Ibus_BYTES_PER_BURST, DMA_Ibus_REQUEST_PER_BURST, 
	    HI16(DMA_Ibus_SRC_BASE), HI16(DMA_Ibus_DST_BASE));
	DMA_Ibus_TD[0] = CyDmaTdAllocate();
	CyDmaTdSetConfiguration(DMA_Ibus_TD[0], 1*DMA_Ibus_BYTES_PER_BURST, DMA_Ibus_TD[0], DMA_Ibus__TD_TERMOUT_EN | TD_INC_DST_ADR);
	CyDmaTdSetAddress(DMA_Ibus_TD[0], LO16((uint32)ADC_therm_DEC_SAMP_PTR), LO16((uint32)adc_dma_array));
	CyDmaChSetInitialTd(DMA_Ibus_Chan, DMA_Ibus_TD[0]);
	CyDmaChEnable(DMA_Ibus_Chan, 1);
    
}

void boost_init() {
    dma_config();
    
    boost_isr_StartEx(isr_boost);
    ibus_isr_StartEx(isr_ibus);
    
    temp_pwm_Start();
    ADC_therm_Start();
    ADC_therm_StartConvert();
    
    bus_voltage_factor = (configuration.r_top + BUSV_R_BOT) / (BUSV_R_BOT * 819 / 1000);
    dtc_constraint = getPeriod() * 9 / 10; // 90% limit
    
    // p =  0.0069 << 16 = 452
    // i =  0.0040 << 16 = 262
    // d = -0.0035 << 16 = -229
    // a = -0.2220 - multiply by -7 = a << 5, then >> 5
    CompInit(&controller_I, 452, 262, -229, -7, 5, 16);
    
    // p = 296447
    // i = 2514
    // d = -281181
    // a = 0.889
    CompInit(&controller_V, 296447, 2514, 0, 0, 0, 0);
    
    temp_pwm_WriteCompare1(0);
    temp_pwm_WriteCompare2(0);
   
    
}

uint16_t getPeriod() {
    return temp_pwm_ReadPeriod() + 1;
}