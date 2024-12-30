

    #ifndef BOOST_H
    #define BOOST_H
    
    #include "comp.h"
    #include <device.h>
    #include <stdint.h>
    
        
    struct boost {
        int64_t v_bridge; //current bridge voltage (mV)
        int64_t v_target; //voltage setpoint
        int64_t v_target2; //slowly changing target voltage
        int64_t v_in; //rectified AC voltage
        
        int64_t i_bridge; //current current  (mA)
        int64_t i_target; //target current
        
        int64_t dtc; //duty cycle
    };
    typedef struct boost BoostStruct;
    
    
    struct dma {
        int16 val;
    };
    typedef struct dma Dma;
    
    extern StructComp controller_V;
    extern StructComp controller_I;
    extern BoostStruct vars;
    
    void dma_config();
    int16 adc_dma_array[1];
    
    void boost_init();
    void setIMax();
    void stop();
    void boost_loop();
    uint16_t getPeriod();

    
#endif