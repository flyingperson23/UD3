#include "lpf_pi.h"

void constrain(float *x, float min, float max){
    if (*x > max) { *x = max; }
    if (*x < min) { *x = min; }
}

void LPF_reset(LPFStruct *controller) {
	controller->E[0] = 0.0f; controller->E[1] = 0.0f;
	controller->LPF_E1[0] = 0.0f; controller->LPF_E1[1] = 0.0f;
	controller->LPF_E2[0] = 0.0f; controller->LPF_E2[1] = 0.0f;
	controller->integrator = 0.0f;
	controller->Y = 0.0f;
}

float LPF_Update(LPFStruct *controller, float error_in) {

	controller->E[0] = controller->Kp*error_in;

	// low-pass filter
	controller->LPF_E1[0] = (controller->E[0])*(1-controller->lpf_pole) + (controller->LPF_E1[1])*(controller->lpf_pole);
	controller->LPF_E2[0] = (controller->LPF_E1[0])*(1.0f-controller->lpf_pole) + (controller->LPF_E2[1])*(controller->lpf_pole);

	// lag, parallel path integrator
	controller->integrator += controller->lag_ki*controller->LPF_E2[0];
	constrain(&(controller->integrator), controller->cmd_lim_min, controller->cmd_lim_max);

	controller->Y = controller->LPF_E2[0] + controller->integrator;
	float tempY = controller->Y;
	constrain(&(controller->Y), controller->cmd_lim_min, controller->cmd_lim_max);
	if (tempY != controller->Y) {controller->cmd_saturation = 1;} else { controller->cmd_saturation = 0; }

	controller->E[1] = controller->E[0];
	controller->LPF_E1[1] = controller->LPF_E1[0];
	controller->LPF_E2[1] = controller->LPF_E2[0];

    return controller->Y;
}