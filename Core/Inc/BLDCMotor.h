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
#include <PID.h>

class BLDCMotor {
public:
	BLDCMotor(uint8_t p, double R, PID PPID, PID VPID);
	virtual ~BLDCMotor();
	int Init();

	double phaseResistance;
	uint8_t poles;
	PID PositionPID;
	PID VelocityPID;

private:
	double elAngleOffset;

	void InverseParkTransform(double elAngle, double direct, double quadrature, double &alpha, double &beta);
	void InverseClarkTransform(double alpha, double beta, double &a, double &b, double &c);
	void SVMGenerator(double alpha, double beta, double &pwma, double &pwmb, double &pwmc);
	double Mech2Elec(double encoderAngle);
	double CalcElAngleOffset();

};

#endif /* BLDCMOTOR_H_ */
