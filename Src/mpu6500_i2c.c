/*
 * MPU9250I2C.c
 *
 *  Created on: Sep 14, 2020
 *      Author: Piotr
 */

#include "mpu6500_i2c.h"
#include "main.h"
#include "i2c.h"
#include <math.h>
#include "discrete_controller.h"

uint8_t mpu6500_Rx_buffer[20]={0};
uint8_t mpu6500_Tx_buffer[20]={0};
uint16_t gyroResolution=0;
#define numberOfSamples 1000
#define dt 0.001
#define numberOfMeasurments 5

HAL_StatusTypeDef mpu6500_i2c_init(I2C_HandleTypeDef* hi2c,uint16_t devAddress,uint8_t gyroRange)
{
	switch(gyroRange)
	{
	case GYRO_RANGE_250DPS:
		{gyroResolution=250;break;}
	case GYRO_RANGE_500DPS:
		{gyroResolution=500;break;}
	case GYRO_RANGE_1000DPS:
		{gyroResolution=1000;break;}
	case GYRO_RANGE_2000DPS:
		{gyroResolution=2000;break;}
	}
	mpu6500_Tx_buffer[0]=gyroRange;
	mpu6500_i2c_setRegister(hi2c, devAddress, GYRO_CONFIG, mpu6500_Tx_buffer);//Set gyro range
	short gyroOffset[3]={GYROX_OFFSET_VAL,GYROY_OFFSET_VAL,GYROZ_OFFSET_VAL};
	for(uint8_t i=0;i<3;i++)
	{
		mpu6500_i2c_setOneAxis(hi2c, devAddress, XG_OFFSET_H+(2*i), &gyroOffset[i]);//set gyro offset for xyz
	}
	return HAL_OK;
}

HAL_StatusTypeDef mpu6500_i2c_setOneAxis(I2C_HandleTypeDef* hi2c,uint16_t devAddress,uint8_t devRegister,short int* pData)
{
	static uint8_t* pointer;
	pointer=(uint8_t*)pData;
	mpu6500_Tx_buffer[1]=*pointer;
	mpu6500_Tx_buffer[0]=*(pointer+1);

	return HAL_I2C_Mem_Write(hi2c, devAddress,devRegister,1, mpu6500_Tx_buffer, 2,1);

}
HAL_StatusTypeDef mpu6500_i2c_setRegister(I2C_HandleTypeDef* hi2c,uint16_t devAddress,uint8_t devRegister,uint8_t* pData)
{
	return HAL_I2C_Mem_Write(hi2c, devAddress, devRegister, I2C_MEMADD_SIZE_8BIT, pData, 1, 1);
}


HAL_StatusTypeDef mpu6500_i2c_getRegister(I2C_HandleTypeDef* hi2c,uint16_t devAddress,uint8_t devRegister,uint8_t* pData)
{
	return HAL_I2C_Mem_Read(hi2c, devAddress, devRegister, I2C_MEMADD_SIZE_8BIT, pData, 1, 1);
}

HAL_StatusTypeDef mpu6500_i2c_getOneAxis(I2C_HandleTypeDef* hi2c,uint16_t devAddress,uint8_t axis,short int* pData)
{
	HAL_I2C_Mem_Read(hi2c, devAddress, axis, I2C_MEMADD_SIZE_8BIT, mpu6500_Rx_buffer, 2, 1);
	*pData=(mpu6500_Rx_buffer[0]<<8)|mpu6500_Rx_buffer[1];

	return HAL_OK;
}

//Returns offsetValue for given axis
short int calculateOneAxisOffSet(uint8_t axis)
{
	long int measurments[numberOfMeasurments]={0};
	short int reading;
	short int offSet=0;
	for(uint16_t j=0;j<numberOfMeasurments;j++)
	{
		for(uint16_t i=0;i<numberOfSamples;i++)
		{
			mpu6500_i2c_getOneAxis(MPU6500_I2C_DEV,MPU6500_I2C_ADDRESS, axis, &reading);
			HAL_Delay(5);
			measurments[j]=measurments[j]+reading;
		}
		measurments[j]=measurments[j]/numberOfSamples;
		HAL_Delay(1000);
	}
	for(uint8_t i=0;i<numberOfMeasurments;i++)
	{
		offSet=offSet+measurments[i];
	}
	return offSet/numberOfMeasurments;
}
//In Radians
HAL_StatusTypeDef mpu6500_i2c_getTheta(I2C_HandleTypeDef* hi2c,uint16_t devAddress,float* theta)
{
	static short int rawRotSpeedX=0,rawAccelY=0,rawAccelZ=0;
	static float rotSpeedX=0,accelTheta=0,gyroTheta=0;
	//Read Gyro
	mpu6500_i2c_getOneAxis(hi2c, devAddress, GYRO_XOUT_H, &rawRotSpeedX);
	//Read Accel
	mpu6500_i2c_getOneAxis(hi2c, devAddress, ACCEL_YOUT_H, &rawAccelY);
	mpu6500_i2c_getOneAxis(hi2c, devAddress, ACCEL_ZOUT_H, &rawAccelZ);
	//Calculate gyro theta
	gyroTheta=rotSpeedX*DT+*theta;
	rotSpeedX=-(((float)rawRotSpeedX/MAX_SIGNED_16_BIT_VALUE)*gyroResolution)*(DEG2RAD);

	//Calculate accel theta
	accelTheta=atan2(rawAccelY,-rawAccelZ);
	//Complementary filter
	*theta=0.98*gyroTheta+accelTheta*0.02;
	return HAL_OK;
}
//Write theta and dtheta both in radian
HAL_StatusTypeDef mpu6500_i2c_getThetaAndDtheta(I2C_HandleTypeDef* hi2c,uint16_t devAddress,float* theta,float* dtheta)
{
	static short int rawRotSpeedX=0,rawAccelY=0,rawAccelZ=0;
		static float rotSpeedX=0,accelTheta=0,gyroTheta=0;
		//Read Gyro
		mpu6500_i2c_getOneAxis(hi2c, devAddress, GYRO_XOUT_H, &rawRotSpeedX);
		//Read Accel
		mpu6500_i2c_getOneAxis(hi2c, devAddress, ACCEL_YOUT_H, &rawAccelY);
		mpu6500_i2c_getOneAxis(hi2c, devAddress, ACCEL_ZOUT_H, &rawAccelZ);
		//Calculate gyro theta
		gyroTheta=rotSpeedX*DT+*theta;
		rotSpeedX=-(((float)rawRotSpeedX/MAX_SIGNED_16_BIT_VALUE)*gyroResolution)*(DEG2RAD);//In radians
		*dtheta=rotSpeedX;

		//Calculate accel theta
		accelTheta=atan2(rawAccelY,-rawAccelZ);//In radians
		//y=0.93250296837*x-1.8631623429*prevX+0.93250296837*pprevX+1.8631623429*prevY-0.86500593674*pprevY;

		//Complementary filter
		*theta=0.98*gyroTheta+accelTheta*0.02;//In radians
		//*theta=band_stop_17_filter(*theta);


		return HAL_OK;
}
