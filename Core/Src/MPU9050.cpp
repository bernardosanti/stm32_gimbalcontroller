/*
 * MPU9050.cpp
 *
 *  Created on: Dec 8, 2022
 *      Author: bernardosantana
 */


#include "MPU9050.h"

MPU9050::MPU9050(uint8_t address, I2C_HandleTypeDef* _hi2c)
{
	_devAddr = address;
	devAddr = (uint16_t)(address << 1);
	hi2c = _hi2c;
}


int MPU9050::Init()
{
	uint8_t ret, check, data;

	ret = HAL_I2C_IsDeviceReady(hi2c, devAddr, 5, TIMEOUT);
	if(ret != HAL_OK)
		return 0;

	HAL_I2C_Mem_Read(hi2c, devAddr, RA_WHO_AM_I, 1, &check, 1 , TIMEOUT);

	if(check == _devAddr)
	{
		//Power management register write all 0's to wake up sensor
		data = 0;
		HAL_I2C_Mem_Write(hi2c,devAddr, RA_PWR_MGMT_1, 1, &data, 1, TIMEOUT);
		//Set data rate of 1KHz by writing SMPRT_DIV register
		data = 0x07;
		HAL_I2C_Mem_Write(hi2c, devAddr, RA_SMPLRT_DIV, 1, &data, 1, TIMEOUT);
		//Configure DLPF to 100 Hz
		data = 0x02;
		HAL_I2C_Mem_Write(hi2c, devAddr, RA_CONFIG, 1, &data, 1, TIMEOUT);
		//Writing both register with 0 to set full scale range
		data = 0x0;
		HAL_I2C_Mem_Write(hi2c, devAddr, RA_ACCEL_CONFIG, 1, &data, 1, TIMEOUT);

		data = 0x0;
		HAL_I2C_Mem_Write(hi2c, devAddr, RA_GYRO_CONFIG, 1, &data, 1, TIMEOUT);
	}
	else
	{
		return 0;
	}

	return 1;
}


int MPU9050::ReadAccel(float &Ax, float &Ay, float &Az)
{
	uint8_t buf[6];
	int16_t ax, ay, az = 0;

	if(HAL_I2C_Mem_Read(hi2c, devAddr, RA_ACCEL_XOUT_H, 1, buf, 6, TIMEOUT) != HAL_OK)
	{
		I2CErrorCounter++;
		return 0;
	}

	//Adding 2 BYTES into 16 bit integer
	ax = ((int16_t)buf[0] << 8 | buf[1]);
	ay = ((int16_t)buf[2] << 8 | buf[3]);
	az = ((int16_t)buf[4] << 8 | buf[5]);

	Ax = (float)ax*9.81/16384.0;
	Ay = (float)ay*9.81/16384.0;
	Az = (float)az*9.81/16384.0;

	return 1;
}

int MPU9050::ReadGyro(float& Gx, float& Gy, float& Gz)
{
	uint8_t buf[6];
	int16_t gx, gy, gz = 0;
	if(HAL_I2C_Mem_Read(hi2c, devAddr, RA_GYRO_XOUT_H, 1, buf, 6, TIMEOUT) != HAL_OK)
	{
		I2CErrorCounter++;
		return 0;
	}

	gx = ((int16_t)buf[0] << 8 | buf[1]);
	gy = ((int16_t)buf[2] << 8 | buf[3]);
	gz = ((int16_t)buf[4] << 8 | buf[5]);

	Gx = (float)gx/131.0;
	Gy = (float)gy/131.0;
	Gz = (float)gz/131.0;

	return 1;
}

int MPU9050::ReadAccelGyro(float& Ax, float& Ay, float& Az, float& Gx, float& Gy, float& Gz)
{
	uint8_t buf[14];
	int16_t ax, ay, az, gx, gy, gz = 0;

	if(HAL_I2C_Mem_Read(hi2c, devAddr, RA_ACCEL_XOUT_H, 1, buf, 14, TIMEOUT) != HAL_OK)
	{
		I2CErrorCounter++;
		return 0;
	}

	ax = ((int16_t)buf[0] << 8 | buf [1]);
	ay = ((int16_t)buf[2] << 8 | buf [3]);
	az = ((int16_t)buf[4] << 8 | buf [5]);

	Ax = (float)ax*9.81/16384.0;
	Ay = (float)ay*9.81/16384.0;
	Az = (float)az*9.81/16384.0;

	gx = ((int16_t)buf[8] << 8 | buf [9]);
	gy = ((int16_t)buf[10] << 8 | buf [11]);
	gz = ((int16_t)buf[12] << 8 | buf [13]);

	Gx = (float)gx/131.0;
	Gy = (float)gy/131.0;
	Gz = (float)gz/131.0;

	return 1;
}
