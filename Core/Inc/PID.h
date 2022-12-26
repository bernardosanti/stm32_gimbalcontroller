/*
 * PID.h
 *
 *  Created on: Dec 3, 2022
 *      Author: bernardosantana
 */

#ifndef PID_H_
#define PID_H_

class PID {
public:
	PID();
	PID(double P, double I, double D);
	virtual ~PID();
	double Calculate(double setPoint, double currentPoint, double dh);
	void ClearITerm();
private:
	double P, I, D;

	double PLim, ILim, DLim, ULim;

	double ITerm;
	double prevError;
};

#endif /* PID_H_ */
