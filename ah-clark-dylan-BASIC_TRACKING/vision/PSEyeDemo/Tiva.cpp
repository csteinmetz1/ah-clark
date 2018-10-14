#include "stdafx.h"
#include <iostream> 
#include <tuple>
#include <string>
#include <cmath>
#include "Tiva.h"

using namespace std;

TivaController::TivaController(double _unitsPerCm, double arm1Cm, double arm2Cm, double xOffsetCm, double yOffsetCm)
{
	unitsPerCm = _unitsPerCm;
	xOffset = unitsPerCm * xOffsetCm;
	yOffset = unitsPerCm * yOffsetCm;
	a1 = unitsPerCm * arm1Cm;
	a2 = unitsPerCm * arm2Cm;

	// initialize arm 
	q1 = 0.0;
	q2 = 0.0;
	x1 = 0.0;
	y1 = 0.0;
	x2 = 0.0;
	y2 = 0.0;
};

void TivaController::resetArm(void) 
{
	setMotor1Angle(0.0);
	setMotor2Angle(0.0);
	updateArmLocation();
};

void TivaController::moveArm(double x, double y, bool negative) 
{
	tuple<double, double> newAngles;
	newAngles = computeKinematics(x, y, negative);
	setMotor1Angle(get<0>(newAngles));
	setMotor2Angle(get<1>(newAngles));
	updateArmLocation();
};

tuple<double, double> TivaController::computeKinematics(double x, double y, bool negative) 
{ 
	double new_q1, new_q2;

	x -= xOffset;
	y -= yOffset;

	if (negative) // negative q2 solution
	{
		new_q2 = -(acos(((pow(x,2) + pow(y,2) - pow(a1,2) - pow(a2,2)) / (2.0 * a1 * a2))));
		new_q1 = atan2(y, x) - atan2((a2 * sin(new_q2)), (a1 + (a2 * cos(new_q2))));
	}
	else 		 // positive q2 solution
	{
		new_q2 =  acos( (pow(x,2) + pow(y,2) - pow(a1,2) - pow(a2,2)) / (2.0 * a1 * a2) );
		new_q1 = atan2(y, x) - atan2((a2 * sin(new_q2)), (a1 + (a2 * cos(new_q2))));
	}
	return make_tuple(new_q1, new_q2);
}

void TivaController::updateArmLocation() 
{
	x1 = a1 * cos(q1) + xOffset;
	y1 = a1 * sin(q1) + yOffset;
	x2 = x1 + (a2 * cos(q1+q2));
	y2 = y1 + (a2 * sin(q1+q2));
}

/*int main() {
	TivaController Tiva = TivaController(1.0, 45.0, 20.0, 33.0, -10.0);
	cout << Tiva.getMotor1Angle() << "\n";
	Tiva.moveArm(5.0, 20.0, false);
	cout << Tiva.getMotor1Angle() << "\n";
}*/