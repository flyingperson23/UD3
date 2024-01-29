

    #ifndef BOOST_H
    #define BOOST_H
    
    #include "pid.h"
    #include <stdint.h>
    
        
    struct boost {
        int v_bridge; //current bridge voltage
        int v_target; //voltage setpoint
        int v_rq; //voltage requested by current controller
        
        int i_bridge; //current current  (*10)
        int i_target; //target current  (*10)
        
        int dtc; //duty cycle
        
    };
    typedef struct boost BoostStruct;
    
    extern PIDStruct controller_V;
    extern PIDStruct controller_I;
    extern BoostStruct vars;
    
    void boost_init();
    void setIMax();
    void stop();
    uint16_t getPeriod();

    
#endif