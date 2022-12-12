/*
 * AS5048B.cpp
 *
 *  Created on: Dec 11, 2022
 *      Author: bernardosantana
 */

#include "AS5048B.h"

AS5048B::AS5048B(uint8_t address, I2C_HandleTypeDef *_hi2c)
{
	_devAddr = address;
	devAddr = (uint16_t)(address << 1);
	hi2c = _hi2c;
}

AS5048B::AS5048B() {
	// TODO Auto-generated constructor stub

}

AS5048B::~AS5048B() {
	// TODO Auto-generated destructor stub
}


int AS5048B::Init()
{
	//uint8_t check;

	if(HAL_I2C_IsDeviceReady(hi2c, devAddr, 5, TIMEOUT_ENC) != HAL_OK)
		return 0;

	/*HAL_I2C_Mem_Read(hi2c, devAddr, RA_WHO_AM_I_ENC, 1, &check, 1 , TIMEOUT_ENC);
	if(check != _devAddr)
		return 0;*/

	return 1;
}

int AS5048B::GetEncoderAngle(float& encAngle)
{
	uint8_t buf[2] = {0};
	if(HAL_I2C_Mem_Read(hi2c, devAddr, RA_ANGLE_ENC, 1, buf, 2 , TIMEOUT_ENC) != HAL_OK)
		return 0;

	uint16_t raw_encAngle = ((int16_t)buf[0] << 6 | (buf[1] & 0x3F));
	encAngle = raw_encAngle * AS5048B_RESOLUTION;

	return 1;
}
