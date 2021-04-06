
#include "discrete_controller.h"
#include "global_variables.h"
#include "tim.h"
#include <stdbool.h>
#include <math.h>
hController disc_controller1;

void discrete_controller_init(hController* hController,TIM_HandleTypeDef* encTimer){
	hController->_32bits_enc=(int*)&(encTimer->Instance->CNT);
	hController->prevX=0;
	hController->x=0;
	hController->prevV=0;
	hController->prevI=0;
	hController->prevTheta=0;
	hController->controlInVolts=0;
	hController->kX=KX;
	hController->kTheta=KTHETA;
	hController->kDx=KDX;
	hController->kDtheta=KDTHETA;
	hController->thetaOffset=0;

}

float* calculate_control_PID(hController* hController,float error)
{

	static float prevError=0;
	static float prevIPart=0;
	static float prevDPart=0;
	static float currIPart=0;
	static float currDPart=0;


	currIPart=prevIPart+prevError*DT*Im;
	currDPart=Dm*Nm*(error-prevError)-prevDPart*(Nm*DT-1);

	hController->controlInVolts=currIPart+currDPart+Pm*error;

	//Clamping
	if(fabs(hController->controlInVolts)>MAX_CONTROL_IN_VOLTS)
	{

		if((error>0 && hController->controlInVolts<0) || (error<0 && hController->controlInVolts>0))//Signs of error and control are different
		{
				prevIPart=currIPart;
		}
		else{prevIPart=prevIPart;}
	}
	else{prevIPart=currIPart;}



	prevError=error;
	prevDPart=currDPart;

	return &hController->controlInVolts;
}

float* calculate_control_K(hController* hController,float theta, float dtheta)
{
		static float F=0;
		static float dX=0;
		//Calculate x in meters based on number of pulses times pulses expressed in meters
		hController->x=(*(hController->_32bits_enc)-INT32_MAX)*ENC_PULSE_IN_METERS;
		x=hController->x;
		//Caltulate derivative based on previous x in meters and current x in meters devided by DT
		dX=(hController->x-hController->prevX)/DT;
		//F according to model in matlab
		F=-(hController->kX*hController->x + hController->kDx*dX + hController->kDtheta*dtheta + hController->kTheta*(theta+hController->thetaOffset));
		//ControlInVolts according to model of dc motor in matlab
		hController->controlInVolts = F*WHEEL_RADIUS*MOTOR_COEF_KI_INV+(MOTOR_COEF_KV)*(dX/WHEEL_RADIUS);
		hController->prevX=hController->x;

		return &hController->controlInVolts;
}

float* discrete_controller_V2PWM(float controlInVolts)
{
	static uint8_t sign=MINUS;
	static float PWM=0;
	if(controlInVolts>0) sign=PLUS;
	else sign=MINUS;

	if(fabs(controlInVolts)>MAX_VOLTS)
	{
		if(sign==PLUS)controlInVolts=MAX_VOLTS;
		else controlInVolts=-MAX_VOLTS;
	}
	else //if(fabs(controlInVolts)<MIN_VOLTS)
	//{
		//controlInVolts=0;
	//}

	PWM=controlInVolts/MAX_POWER_SUPP;
	return &PWM;
}
void change_thetaOffset_to(hController* hController,float newOffset)
{
	hController->thetaOffset=newOffset;
}
float band_stop_17_filter(float x)
{
	static float prevX =0,pprevX=0;
	static float prevY=0,pprevY=0,y=0;
	//y=0.93250296837*x-1.8631623429*prevX+0.93250296837*pprevX+1.8631623429*prevY-0.86500593674*pprevY;
	y=0.903161052261089*x-1.805069543170061*prevX+0.903161052261089*pprevX+1.805069543170061 *prevY-0.806322104522179*pprevY;
	pprevX=prevX;
	prevX=x;
	pprevY=prevY;
	prevY=y;
	return y;
}



