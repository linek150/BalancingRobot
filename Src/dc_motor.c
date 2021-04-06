/*
 * dc_motor.c
 *
 *  Created on: Sep 18, 2020
 *      Author: Piotr
 */

#include "dc_motor.h"
#include "gpio.h"


hdc_motor hdc_motor1;
hdc_motor hdc_motor2;
HAL_StatusTypeDef dc_motor_init(hdc_motor* dc_motor,GPIO_TypeDef* dir1Port,uint16_t dir1Pin,GPIO_TypeDef* dir2Port,uint16_t dir2Pin,TIM_HandleTypeDef* pwmTimer,uint8_t pwmTimerChan,TIM_HandleTypeDef* encTimer )
{
	dc_motor->pwmTimerChan=pwmTimerChan;
	dc_motor->pwmMaxValue=(pwmTimer->Instance->ARR);
	dc_motor->dir1Pin=dir1Pin;
	dc_motor->dir2Pin=dir2Pin;
	dc_motor->dir1Port=dir1Port;
	dc_motor->dir2Port=dir2Port;
	dc_motor->pwmTimer=pwmTimer;
	dc_motor->encTimer=encTimer;
	dc_motor->encCount=&(encTimer->Instance->CNT);
	*(dc_motor->encCount)=INT32_MAX;
	HAL_TIM_PWM_Start(pwmTimer, pwmTimerChan);
	return HAL_OK;
}
HAL_StatusTypeDef dc_motor_runSpeed(hdc_motor* dc_motor,float signedPWM_0_1)
{
	static uint16_t speed;
	if(signedPWM_0_1==0)
	{

		dc_motor_setDir(dc_motor, STOP);
	}
	else if(signedPWM_0_1>0)
	{
		dc_motor_setDir(dc_motor, FORWARD);

	}
	else
	{
		dc_motor_setDir(dc_motor,BACKWARD);
		signedPWM_0_1=-signedPWM_0_1;
	}
	speed=signedPWM_0_1*dc_motor->pwmMaxValue;
	__HAL_TIM_SET_COMPARE(dc_motor->pwmTimer,dc_motor->pwmTimerChan,speed);
	return HAL_OK;
}
HAL_StatusTypeDef dc_motor_setDir(hdc_motor* dc_motor,uint8_t direction)
{
	if(direction == FORWARD)
	{
		HAL_GPIO_WritePin(dc_motor->dir1Port, dc_motor->dir1Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(dc_motor->dir2Port, dc_motor->dir2Pin, GPIO_PIN_RESET);
	}
	else if(direction == BACKWARD)
	{
		HAL_GPIO_WritePin(dc_motor->dir1Port, dc_motor->dir1Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dc_motor->dir2Port, dc_motor->dir2Pin, GPIO_PIN_SET);
	}
	else if(direction==STOP)
	{
		HAL_GPIO_WritePin(dc_motor->dir1Port, dc_motor->dir1Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dc_motor->dir2Port, dc_motor->dir2Pin, GPIO_PIN_RESET);
	}
	return HAL_OK;
}

