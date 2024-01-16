

    #ifndef BOOST_H
    #define BOOST_H
    
    #include "lpf_pi.h"
    
        
    struct boost {
        int v_bridge; //current bridge voltage
        int v_target; //target voltage
        
        int i_bridge; //current current
        int i_target; //target current;
        
        int dtc; //duty cycle  (* 10^8)
        
        int dtc_v; //current-based dtc change  (* 10^8)
        int dtc_i; //voltage-based dtc change  (* 10^8)
        
    };
    typedef struct boost BoostStruct;
    
    extern LPFStruct controller_V;
    extern LPFStruct controller_I;
    extern BoostStruct vars;
    
    void boost_init();
    void setIMax();
    void stop();

    
#endif