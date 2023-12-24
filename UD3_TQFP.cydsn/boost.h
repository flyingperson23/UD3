#include <stdint.h>

    #ifndef BOOST_H
    #define BOOST_H
    
    #include "lpf_pi.h"
    
        
    typedef struct{
        float v_bridge; //current bridge voltage
        float v_target; //target voltage
        
        float i_bridge; //current current
        float i_target; //target current;
        
        float dtc; //duty cycle
        
        float dtc_v; //current-based dtc change
        float dtc_i; //voltage-based dtc change
        
    } BoostStruct;
            
    extern LPFStruct controller_V;
    extern LPFStruct controller_I;
    extern BoostStruct vars;
    
    void boost_init();
    void setIMax();
    void stop();

    
#endif