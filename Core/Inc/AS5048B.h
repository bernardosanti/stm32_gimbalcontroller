/*
 * AS5048B.h
 *
 *  Created on: Dec 11, 2022
 *      Author: bernardosantana
 */

#ifndef SRC_AS5048B_H_
#define SRC_AS5048B_H_

#include <stdint.h>
#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RA_WHO_AM_I_ENC		0x15
#define RA_ANGLE_ENC		0xFE

#define TIMEOUT_ENC 100

class AS5048B {
public:
	AS5048B(uint8_t devAddr, I2C_HandleTypeDef* hi2c);
	AS5048B();
	virtual ~AS5048B();
	int Init();
	int GetEncoderAngle(int16_t& encAngle);

private:
    uint8_t _devAddr;
    uint16_t devAddr;
    I2C_HandleTypeDef* hi2c;
};

#ifdef __cplusplus
}
#endif

#endif /* SRC_AS5048B_H_ */
