

    #ifndef BOOST_H
    #define BOOST_H
    
    #include "pid.h"
    #include <device.h>
    #include <stdint.h>
    
        
    struct boost {
        int v_bridge; //current bridge voltage
        int v_target; //voltage setpoint
        int v_target2; //slowly changing target voltage
        int v_in; //rectified AC voltage
        
        int i_bridge; //current current  (*10)
        int i_target; //target current  (*10)
        
        int dtc; //duty cycle
    };
    typedef struct boost BoostStruct;
    
    
    struct dma {
        int16 val;
    };
    typedef struct dma Dma;
    
    extern PIDStruct controller_V;
    extern PIDStruct controller_I;
    extern BoostStruct vars;
    
    void dma_config();
    int16 adc_dma_array[4];
    
    void boost_init();
    void setIMax();
    void stop();
    void boost_loop();
    uint16_t getPeriod();

    
#endif