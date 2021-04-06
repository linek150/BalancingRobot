/*
 * mpu9250I2C.h
 *
 *  Created on: Sep 14, 2020
 *      Author: Piotr
 */

#ifndef INC_MPU6500_I2C_H_
#define INC_MPU6500_I2C_H_

#include "main.h"
#define MPU6500_I2C_ADDRESS (0x68<<1)
#define MPU6500_I2C_DEV &hi2c1
#define DT 0.001
#define MAX_SIGNED_16_BIT_VALUE 32767
#define DEG2RAD 0.01745329252


#define GYROX_OFFSET_VAL 134
#define GYROY_OFFSET_VAL -42
#define GYROZ_OFFSET_VAL -11


#define GYRO_RANGE_250DPS 0
#define GYRO_RANGE_500DPS 8
#define GYRO_RANGE_1000DPS 16
#define GYRO_RANGE_2000DPS 24

#define PI 3.1415


// MPU6500 registers

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13 //OK  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14 //OK
#define YG_OFFSET_H      0x15 //OK
#define YG_OFFSET_L      0x16 //OK
#define ZG_OFFSET_H      0x17 //OK
#define ZG_OFFSET_L      0x18 //OK
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C //OK
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B //OK
#define ACCEL_XOUT_L     0x3C //OK
#define ACCEL_YOUT_H     0x3D //OK
#define ACCEL_YOUT_L     0x3E //OK
#define ACCEL_ZOUT_H     0x3F //OK
#define ACCEL_ZOUT_L     0x40 //OK
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43 //OK
#define GYRO_XOUT_L      0x44 //OK
#define GYRO_YOUT_H      0x45 //OK
#define GYRO_YOUT_L      0x46 //OK
#define GYRO_ZOUT_H      0x47 //OK
#define GYRO_ZOUT_L      0x48 //OK
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU6500 0x75 // Should return 0x70
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E




extern uint8_t mpu6500_Rx_buffer[20];
extern uint8_t mpu6500_Tx_buffer[20];
HAL_StatusTypeDef mpu6500_i2c_init(I2C_HandleTypeDef* hi2c,uint16_t devAddress,uint8_t gyroRange);
HAL_StatusTypeDef mpu6500_i2c_setRegister(I2C_HandleTypeDef* hi2c,uint16_t devAddress,uint8_t devRegister,uint8_t* pData);
HAL_StatusTypeDef mpu6500_i2c_getRegister(I2C_HandleTypeDef* hi2c,uint16_t devAddress,uint8_t devRegister,uint8_t* pData);
HAL_StatusTypeDef mpu6500_i2c_getOneAxis(I2C_HandleTypeDef* hi2c,uint16_t devAddress,uint8_t axis,short int* pData);
HAL_StatusTypeDef mpu6500_i2c_setOneAxis(I2C_HandleTypeDef* hi2c,uint16_t devAddress,uint8_t devRegister,short int* pData);
HAL_StatusTypeDef mpu6500_i2c_getTheta(I2C_HandleTypeDef* hi2c,uint16_t devAddress,float* theta);
HAL_StatusTypeDef mpu6500_i2c_getThetaAndDtheta(I2C_HandleTypeDef* hi2c,uint16_t devAddress,float* theta,float* dtheta);
short int calculateOneAxisOffSet(uint8_t axis);


#endif /* INC_MPU6500_I2C_H_ */
