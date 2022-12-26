/*
 * BLDCMotor.cpp
 *
 *  Created on: Dec 3, 2022
 *      Author: bernardosantana
 */

#include "BLDCMotor.h"

BLDCMotor::BLDCMotor(uint8_t p, double R, PID PPID, PID VPID, AS5048B *_encoder, TIM_HandleTypeDef *_htim) {
	poles = p;
	phaseResistance = R;
	PositionPID = PPID;
	VelocityPID = VPID;
	encoder = _encoder;
	htim = _htim;

	SetPWM(10.0, 20.0, 70.0);
}

BLDCMotor::~BLDCMotor() {
	// TODO Auto-generated destructor stub
}

int BLDCMotor::Init()
{
	return CalcElAngleOffset();
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

	//SVPWM algorithm
	// 1. Determine which sector the motor is on
	//uint8_t sector = floor(elAngle / _PI_3) + 1;

	// 2. Calculate Vref magnitude
	//double Uout = sqrt(pow(alpha,2) + pow(beta,2));

    // 3. Calculate the duty cycles
    //double T1 = _SQRT3 * sin(sector * _PI_3 - elAngle) * Uout;
    //double T2 = _SQRT3 * sin(elAngle - (sector-1.0) * _PI_3) * Uout;
    //double T0 = 1 - T1 - T2;

    /*double Ta,Tb,Tc;
    switch(sector){
      case 1:
        Ta = T1 + T2 + T0/2;
        Tb = T2 + T0/2;
        Tc = T0/2;
        break;
      case 2:
        Ta = T1 +  T0/2;
        Tb = T1 + T2 + T0/2;
        Tc = T0/2;
        break;
      case 3:
        Ta = T0/2;
        Tb = T1 + T2 + T0/2;
        Tc = T2 + T0/2;
        break;
      case 4:
        Ta = T0/2;
        Tb = T1+ T0/2;
        Tc = T1 + T2 + T0/2;
        break;
      case 5:
        Ta = T2 + T0/2;
        Tb = T0/2;
        Tc = T1 + T2 + T0/2;
        break;
      case 6:
        Ta = T1 + T2 + T0/2;
        Tb = T0/2;
        Tc = T1 + T0/2;
        break;
      default:
       // possible error state
        Ta = 0;
        Tb = 0;
        Tc = 0;
    }*/

    // calculate the phase voltages and center
    /*pwma = Ta * (VLim / VDC);
    pwmb = Tb * (VLim / VDC);
    pwmc = Tc * (VLim / VDC);*/
}


void BLDCMotor::SetPWM(double pwma, double pwmb, double pwmc)
{
	// Map the PWM to the timer presets
	double ccra = htim->Instance->ARR * pwma;
	double ccrb = htim->Instance->ARR * pwmb;
	double ccrc = htim->Instance->ARR * pwmc;

	htim->Instance->CCR1 = ccra;
	htim->Instance->CCR2 = ccrb;
	htim->Instance->CCR3 = ccrc;
}

double BLDCMotor::Mech2Elec(double encoderAngle)
{
	return (encoderAngle * (poles / 2) + elAngleOffset);
}

int BLDCMotor::CalcElAngleOffset()
{
	// move motor X degrees to one side and to the other
	int res = 1;
	return res;
}

double BLDCMotor::GetElAngle()
{
	return elAngle;
}

void BLDCMotor::UpdateElAngle()
{
	float encoderAngle = 0.0;
	encoder->GetEncoderAngle(encoderAngle);
	elAngle = Mech2Elec(encoderAngle);
}
