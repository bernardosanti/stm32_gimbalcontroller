/*
 * BLDCMotor.h
 *
 *  Created on: Dec 3, 2022
 *      Author: bernardosantana
 */

#ifndef BLDCMOTOR_H_
#define BLDCMOTOR_H_

#include <stdint.h>
#include <math.h>
#include "PID.h"
#include "AS5048B.h"
#include "stm32f4xx.h"


#define _PI_3 1.0471975511965977461542144610932
#define _2PI  6.283185307179586
#define _SQRT3_2 0.866025403784438646763723170752
#define _2_SQRT3 1/_SQRT3_2
#define _SQRT3 1.7320508075688772935274463415059

class BLDCMotor {
public:

	enum ControllerType_e : uint8_t
	{
		VOLTAGE	= 0,
		CURRENT = 1
	};

	BLDCMotor(uint8_t p, double R, AS5048B *encoder, TIM_HandleTypeDef *htim);
	virtual ~BLDCMotor();
	int Init();
	void SetIqRef(double vref);
	void FOCLoop();

	// We use this to implement a BEMF observer and perform compensation
	double phaseResistance;
	double phaseInductance;
	uint8_t poles;

	double VLim = 12.0;
	double VDC = 12.0;

	AS5048B *encoder;
	TIM_HandleTypeDef *htim;

	ControllerType_e CtlType;

private:
	double elAngle;
	double elAngleOffset;
	double IqRef;

	void InverseParkTransform(double elAngle, double direct, double quadrature, double &alpha, double &beta);
	void InverseClarkTransform(double alpha, double beta, double &a, double &b, double &c);
	void SVMGenerator(double alpha, double beta, double &pwma, double &pwmb, double &pwmc);
	void SetPWM(double pwma, double pwmb, double pwmc);
	double Mech2Elec(double encoderAngle);
	double GetElAngle(void);
	void UpdateElAngle(void);
	int CalcElAngleOffset();

};

#endif /* BLDCMOTOR_H_ */
