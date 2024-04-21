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
    vars.i_target = 0;
    vars.v_target2 = 0;
    
    controller_V.e[0] = 0;
    controller_V.e[1] = 0;
    controller_V.e[2] = 0;
    controller_V.m[0] = 0;
    controller_V.m[1] = 0;
    
    controller_I.e[0] = 0;
    controller_I.e[1] = 0;
    controller_I.e[2] = 0;
    controller_I.m[0] = 0;
    controller_I.m[1] = 0;
}

uint32_t read_bus_mv2(uint16_t raw_adc) {
	uint32_t bus_voltage;
	bus_voltage = ((configuration.r_top + BUSV_R_BOT) * raw_adc) / (BUSV_R_BOT * 819 / 1000);
	return bus_voltage;
}

CY_ISR(isr_ibus) {
    int avg = adc_dma_array[0]+adc_dma_array[1]+adc_dma_array[2]+adc_dma_array[3];
    avg = avg >> 2;
    avg += 7;
    if (avg < 0) {
        avg = 0;
    }
    //
    
    //avg *= 10;
    
    //
    if(configuration.ct2_type==CT2_TYPE_CURRENT){
	    vars.i_bridge = avg * params.idc_ma_count / 100;
    }else{
        vars.i_bridge = (avg - params.ct2_offset_cnt) * params.idc_ma_count / 100;
    }
    
    
    
    boost_loop();
}

CY_ISR(isr_boost) {
    //boost_loop();
}

int counter = 0;
int vloop_cnt = 0;
void boost_loop() {
    // set vars
    vars.v_bridge = read_bus_mv2(ADC_active_sample_buf[0].v_bus) / 1000;
    vars.v_in = read_bus_mv2(ADC_active_sample_buf[0].v_batt) / 1000;
    //vars.v_bridge = read_bus_mv2(ADC_active_sample_buf[0].v_bus) / 100;
    //vars.v_in = read_bus_mv2(ADC_active_sample_buf[0].v_batt) / 100;
   
    //check limits
    if (vars.v_target * tt.n.batt_v.divider * 10 > tt.n.batt_v.value * 12 && vars.v_bridge * 10 > 12 * vars.v_target) {
        if (sysfault.ov == 0) {
            alarm_push(ALM_PRIO_CRITICAL, "Bus: Overvoltage ", vars.v_bridge);
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
    } else if (vars.v_target > 20) {
        if (vars.v_target2 < vars.v_in) {
            vars.v_target2 = vars.v_in;
        }
        if (vars.v_target2 > vars.v_target) {
            vars.v_target2--;
        } else if (vars.v_target2 < vars.v_target) {
            counter++;
            if (counter % 64 == 0) {
                vars.v_target2++;
            }
        }
        constrain(&vars.v_target2, 0, vars.v_bridge + 150);
        
        //vars.i_target = vars.v_in * controller_V.Kp / 100;
        vars.dtc = PID_Update(&controller_I, vars.i_target - vars.i_bridge) / 1000;
        if (vars.v_bridge + 10 >= vars.v_target2) vars.dtc = 0;
        
        vloop_cnt++;
        
        if(vloop_cnt >= 60){
            vloop_cnt = 0;
            vars.i_target = PID_Update(&controller_V, vars.v_target2 - vars.v_bridge) * vars.v_in / 1000000;
            constrain(&vars.i_target, -configuration.max_dc_curr, configuration.max_dc_curr);
            if (vars.v_bridge + 10 >= vars.v_target2) vars.i_target = 0;
            if (vars.i_target > 0 && vars.i_target < 10) vars.i_target = 0;
            if (vars.i_target < 0 && vars.i_target > -10) vars.i_target = 0;
        }
        
        constrain(&vars.dtc, 0, getPeriod() * 9 / 10); // max 80% dtc
        
        // write dtc
        temp_pwm_WriteCompare1(vars.dtc);
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
	CyDmaTdSetConfiguration(DMA_Ibus_TD[0], 4*DMA_Ibus_BYTES_PER_BURST, DMA_Ibus_TD[0], DMA_Ibus__TD_TERMOUT_EN | TD_INC_DST_ADR);
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
    
    controller_V.Ki = 1;
    controller_V.Kp = 10000; // prev 1
    controller_V.Kd = 0;
    
    
    controller_I.Ki = 3; // prev 10   25
    controller_I.Kp = 10; // prev 7
    controller_I.Kd = 0;
    
}

uint16_t getPeriod() {
    return temp_pwm_ReadPeriod() + 1;
}