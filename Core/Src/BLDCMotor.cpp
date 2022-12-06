/*
 * BLDCMotor.cpp
 *
 *  Created on: Dec 3, 2022
 *      Author: bernardosantana
 */

#include "BLDCMotor.h"

BLDCMotor::BLDCMotor(uint8_t p, double R, PID PPID, PID VPID) {
	poles = p;
	phaseResistance = R;
	PositionPID = PPID;
	VelocityPID = VPID;
}

BLDCMotor::~BLDCMotor() {
	// TODO Auto-generated destructor stub
}

int BLDCMotor::Init()
{
	elAngleOffset = CalcElAngleOffset();
}

void BLDCMotor::InverseParkTransform(double elAngle, double direct, double quadrature, double &alpha, double &beta)
{
	double cosElAngle = cos(elAngle);
	double sinElAngle = sin(elAngle);

	alpha = (cosElAngle * direct) - (sinElAngle * quadrature);
	beta = (cosElAngle * quadrature) + (sinElAngle * direct);
}

void BLDCMotor::InverseClarkTransform(double alpha, double beta, double &a, double &b, double &c)
{
	double _one_two = alpha * 0.5;
	double _sqrt3_two = sqrt(3) / 2;

	a = alpha;
	b = -_one_two + _sqrt3_two;
	c = -_one_two - _sqrt3_two;
}

void BLDCMotor::SVMGenerator(double alpha, double beta, double &pwma, double &pwmb, double &pwmc)
{
	double a, b, c = 0;
	InverseClarkTransform(alpha, beta, a, b, c);
	double maxVal = fmax(fmax(a,b),c);
	double minVal = fmin(fmin(a,b),c);

	double minmax = (maxVal + minVal) / 2;

	pwma = (a + minmax) * (2 / sqrt(3));
	pwmb = (b + minmax) * (2 / sqrt(3));
	pwmc = (c + minmax) * (2 / sqrt(3));
}

double BLDCMotor::Mech2Elec(double encoderAngle)
{
	return (encoderAngle * (poles / 2) + elAngleOffset);
}

double BLDCMotor::CalcElAngleOffset()
{
	// move motor X degrees to one side and to the other
	double _ElAngleOffset;
	return _ElAngleOffset;
}
