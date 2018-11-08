#ifndef TIVA_H
#define TIVA_H

#include <tuple>
#include <string>
#include <iostream> 
#include <cmath>
#include "Puck.h"

class TivaController
{
public:
	// getter methods
	double getMotor1AngleRadians() const;
	double getMotor2AngleRadians() const;
	double getMotor1AngleDegrees() const;
	double getMotor2AngleDegrees() const;
	Vec_double getArm1Location() const;
	Vec_double getArm2Location() const;
	double getArm1Length() const;
	double getArm2Length() const;
	double getxOffset() const;
	double getyOffset() const;

	// setter methods
	void setMotor1AngleRadians(double new_q1);
	void setMotor2AngleRadians(double new_q2);
	void setMotor1AngleDegrees(double new_q1);
	void setMotor2AngleDegrees(double new_q2);
	void setXOffsetCm(double new_xOffsetCm);
	void setYOffsetCm(double new_yOffsetCm);
	void setArm1Cm(double new_arm1Cm);
	void setArm2Cm(double new_arm2Cm);

	// arm movement methods
	void resetArm();
	void updateArmLocation();
	void moveArm(Vec_double point, bool negative);
	static std::vector<Vec_double> computePath(Vec_double start, Vec_double end, int steps);
	static std::vector<Vec_double> computeHitPath(std::vector<Vec_double> trajectory, Vec_double targetPoint,
		double yhit=20, double xlim=10, double ylim=10, int steps=250);
	std::tuple<double, double> computeKinematics(Vec_double point, bool negative);

	// drawing
	static std::vector<Vec_double> Circle(double radius, int steps, Vec_double offset);

	// Constructor
	TivaController(double _unitsPerCm, double arm1Cm, double arm2Cm, double xOffsetCm, double yOffsetCm);

private:
	double q1;
	double q2;
	Vec_double arm1Pos;
	Vec_double arm2Pos;

	double unitsPerCm;
	double xOffset;
	double yOffset;
	double a1;
	double a2;
};

#endif