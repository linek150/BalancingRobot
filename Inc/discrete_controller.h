/*
 * discrete_controller.h
 *
 *  Created on: Sep 23, 2020
 *      Author: Piotr
 */

#ifndef INC_DISCRETE_CONTROLLER_H_
#define INC_DISCRETE_CONTROLLER_H_
#define MINUS 1
#define PLUS 2
#define MAX_POWER_SUPP 12.3
#define MAX_VOLTS 6.f
#define MIN_VOLTS 1.4f
#define DT 0.001f
#define ENC_PULSE_IN_METERS 0.00016790175f
#define KX -0.05
#define KTHETA -35.0457
#define KDX -2.0735
#define KDTHETA -9.6956
#define KI -27
#define Pm .2
#define Im 7.8
#define Dm 0.008
#define Nm 360.587799368
#define Tsm 0.001
#define MAX_CONTROL_IN_VOLTS 5.8
#define WHEEL_RADIUS 0.04 // In meters
#define MOTOR_RESITANCE 1.8750 // IN ohms
#define MOTOR_COEF_KI_INV 54.6278
#define MOTOR_COEF_KV 0.2918
#define NUM_OF_PPR 1496.88
#include "tim.h"

typedef struct{
	int* _32bits_enc;
	float x;
	float prevX;
	float theta;
	float prevI;
	float prevV;
	float prevTheta;
	float controlInVolts;
	float kTheta;
	float kDtheta;
	float kX;
	float kDx;
	float thetaOffset;
}hController;

extern hController disc_controller1;
void discrete_controller_init(hController* hController,TIM_HandleTypeDef* encTimer);
void change_thetaOffset_to(hController* hController,float newOffset);
float* calculate_control_PID(hController* hController,float error);
float* calculate_control_K(hController* hController,float theta, float dtheta);
float* discrete_controller_V2PWM(float controlInVolts);
float band_stop_17_filter(float x);

#endif /* INC_DISCRETE_CONTROLLER_H_ */
