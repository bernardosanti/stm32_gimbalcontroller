/*
 * BLDCMotor.cpp
 *
 *  Created on: Dec 3, 2022
 *      Author: bernardosantana
 */

#include "BLDCMotor.h"

BLDCMotor::BLDCMotor(uint8_t p, double R, AS5048B *_encoder, TIM_HandleTypeDef *_htim)
{
	poles = p;
	phaseResistance = R;
	encoder = _encoder;
	htim = _htim;
	CtlType = BLDCMotor::VOLTAGE;

	SetPWM(0.0, 0.0, 0.0);
}

BLDCMotor::~BLDCMotor()
{
	// TODO Auto-generated destructor stub
}

int BLDCMotor::Init()
{
	return CalcElAngleOffset();
}

void BLDCMotor::SetIqRef(double _iqref)
{
	IqRef = _iqref;
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

	a = alpha;
	b = -_one_two + _SQRT3_2;
	c = -_one_two - _SQRT3_2;
}

void BLDCMotor::SVMGenerator(double alpha, double beta, double &pwma, double &pwmb, double &pwmc)
{
	double a, b, c = 0;
	InverseClarkTransform(alpha, beta, a, b, c);
	double maxVal = fmax(fmax(a,b),c);
	double minVal = fmin(fmin(a,b),c);

	double minmax = (maxVal + minVal) / 2;

	pwma = (a + minmax) * _2_SQRT3;
	pwmb = (b + minmax) * _2_SQRT3;
	pwmc = (c + minmax) * _2_SQRT3;

	//SVPWM algorithm
	// 1. Determine which sector the motor is on
	//uint8_t sector = floor(elAngle / _PI_3) + 1;

	// 2. Calculate Vref magnitude
	//double Uout = sqrt(pow(alpha,2) + pow(beta,2));

    // 3. Calculate the duty cycles
    /*double T1 = _SQRT3 * sin(sector * _PI_3 - elAngle) * Uout;
    double T2 = _SQRT3 * sin(elAngle - (sector-1.0) * _PI_3) * Uout;
    double T0 = 1 - T1 - T2;

    double Ta,Tb,Tc;
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
    }

    // calculate the phase voltages and center
    pwma = Ta * (VLim / VDC);
    pwmb = Tb * (VLim / VDC);
    pwmc = Tc * (VLim / VDC);*/
}


void BLDCMotor::SetPWM(double pwma, double pwmb, double pwmc)
{
	// Limit input pwm to 1
	pwma > 1.0 ? pwma = 1.0 : pwma < 0.0 ? pwma = 0.0 : pwma = pwma;
	pwmb > 1.0 ? pwmb = 1.0 : pwmb < 0.0 ? pwmb = 0.0 : pwmb = pwmb;
	pwmc > 1.0 ? pwmc = 1.0 : pwmc < 0.0 ? pwmc = 0.0 : pwmc = pwmc;

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
	float initAngle = 0.0;
	encoder->GetEncoderAngle(initAngle);
	// move motor X degrees to one side and to the other
	for(uint16_t i = 0; i < 500; i++)
	{
		elAngle += _2PI * i / 500;
		double alpha, beta = 0;
		InverseParkTransform(elAngle, 0.0, 3.0, alpha, beta);
		double pwma, pwmb, pwmc = 0.0;
		SVMGenerator(alpha, beta, pwma, pwmb, pwmc);
		SetPWM(pwma, pwmb, pwmc);
	}
	float topAngle = 0.0;
	encoder->GetEncoderAngle(topAngle);

	for(uint16_t i = 0; i < 1000; i++)
	{
		elAngle += _2PI - (2 *_2PI * i / 1000);
		double alpha, beta = 0;
		InverseParkTransform(elAngle, 0.0, 3.0, alpha, beta);
		double pwma, pwmb, pwmc = 0.0;
		SVMGenerator(alpha, beta, pwma, pwmb, pwmc);
		SetPWM(pwma, pwmb, pwmc);
	}
	float bottomAngle = 0.0;
	encoder->GetEncoderAngle(bottomAngle);

	if(topAngle > initAngle)
	{
		//CCW
	}
	else
	{
		//CW
	}
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

void BLDCMotor::FOCLoop()
{
	UpdateElAngle();

	double alpha, beta = 0;
	if(CtlType == BLDCMotor::VOLTAGE)
	{
		double VqRef = IqRef * phaseResistance;
		InverseParkTransform(elAngle, 0.0, VqRef, alpha, beta);
	}
	else if(CtlType == BLDCMotor::CURRENT)
	{
		InverseParkTransform(elAngle, 0.0, IqRef, alpha, beta);
	}
	double pwma, pwmb, pwmc = 0.0;
	SVMGenerator(alpha, beta, pwma, pwmb, pwmc);
	SetPWM(pwma, pwmb, pwmc);
}
