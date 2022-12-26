/*
 * PID.cpp
 *
 *  Created on: Dec 3, 2022
 *      Author: bernardosantana
 */

#include "PID.h"

PID::PID() : PID(1.0, 0.0, 0.0)
{


}

PID::PID(double _P, double _I, double _D)
{
	P = _P;
	I = _I;
	D = _D;
}

PID::~PID() {
	// TODO Auto-generated destructor stub
}

double PID::Calculate(double setPoint, double currentPoint, double dh)
{
	double error = setPoint - currentPoint;

	double PTerm = P * error;
	PTerm = PTerm > PLim ? PLim : PTerm < -PLim ? -PLim : PTerm;

	// Anti-Windup
	ITerm += I * (error * dh);
	ITerm = ITerm > ILim ? ILim : ITerm < -ILim ? -ILim : ITerm;

	double DTerm = D * ((error - prevError) / dh);
	DTerm = DTerm > DLim ? DLim : DTerm < -DLim ? -DLim : DTerm;

	// Limit Output
	double u = PTerm + ITerm + DTerm;
	u = u > ULim ? ULim : u < -ULim ? -ULim : u;

	return u;
}

void PID::ClearITerm()
{
	ITerm = 0.0;
}
