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
#define _SQRT3_2 0.866025403784438646763723170752
#define _SQRT3 1.7320508075688772935274463415059

class BLDCMotor {
public:
	BLDCMotor(uint8_t p, double R, PID PPID, PID VPID, AS5048B *encoder, TIM_HandleTypeDef *htim);
	virtual ~BLDCMotor();
	int Init();

	// We use this to implement a BEMF observer and perform compensation
	double phaseResistance;
	double phaseInductance;
	uint8_t poles;
	PID PositionPID;
	PID VelocityPID;
	AS5048B *encoder;
	double VLim = 12.0;
	double VDC = 12.0;
	TIM_HandleTypeDef *htim;

private:
	double elAngle;
	double elAngleOffset;

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
