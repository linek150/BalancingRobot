/*
 * dc_motor.h
 *
 *  Created on: Sep 18, 2020
 *      Author: Piotr
 */

#ifndef INC_DC_MOTOR_H_
#define INC_DC_MOTOR_H_
#include "stm32f4xx_hal.h"
#define FORWARD 1
#define BACKWARD 2
#define STOP 0
#define ENC_COUNTER_TYPE uint32_t


typedef struct {
	uint8_t pwmTimerChan;
	uint16_t pwmMaxValue;
	uint16_t dir1Pin,dir2Pin;
	GPIO_TypeDef* dir1Port;
	GPIO_TypeDef* dir2Port;
	TIM_HandleTypeDef* pwmTimer;
	TIM_HandleTypeDef* encTimer;
	volatile ENC_COUNTER_TYPE* encCount;
}hdc_motor;

extern hdc_motor hdc_motor1;
extern hdc_motor hdc_motor2;
HAL_StatusTypeDef dc_motor_init(hdc_motor* dc_motor,GPIO_TypeDef* dir1Port,uint16_t dir1Pin,GPIO_TypeDef* dir2Port,uint16_t dir2Pin,TIM_HandleTypeDef* pwmTimer,uint8_t pwmTimerChan,TIM_HandleTypeDef* encTimer );
HAL_StatusTypeDef dc_motor_runSpeed(hdc_motor* dc_motor,float signedPWMPercent);
HAL_StatusTypeDef dc_motor_setDir(hdc_motor* dc_motor,uint8_t direction);
#endif /* INC_DC_MOTOR_H_ */
