#ifndef LPF_PI_H
#define LPF_PI_H

typedef struct{
	float lpf_pole;
	float lag_ki;
	float Kp;
	float cmd_lim_min, cmd_lim_max;

	float E[2];
	float LPF_E1[2];
	float LPF_E2[2];

	float integrator;
	float Y;

	unsigned char cmd_saturation;

	} LPFStruct;


float LPF_Update(LPFStruct *controller, float error_in);

void LPF_Reset(LPFStruct *controller);

void constrain(float *x, float min, float max);

#endif
