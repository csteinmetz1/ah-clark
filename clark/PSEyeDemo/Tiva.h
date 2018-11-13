#ifndef TIVA_H
#define TIVA_H

#include <tuple>
#include <string>
#include <iostream> 
#define _USE_MATH_DEFINES
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
		std::tuple<double,double> computeKinematics(Vec_double point, bool negative);

		// path methods
		static std::tuple<Vec_double, int> findBlockPoint(std::vector<Vec_double> trajectory, double yblock);
		std::vector<Vec_double> computeLinearPath(Vec_double start, Vec_double end, int steps);
		
		std::vector<Vec_double> computeBlockPath(std::vector<Vec_double> trajectory, double sampleTime, double yblock);
		//std::vector<Vec_double> computeHitPath(std::vector<Vec_double> trajectory);
		std::vector<Vec_double> computeBlockAndHitPath(std::vector<Vec_double> trajectory, Vec_double targetPoint, double sampleTime, double yblock, double stepFactor);
		std::vector<Vec_double> computeSwingPath(std::vector<Vec_double> trajectory, Vec_double targetPoint, double sampleTime, double yblock, double stepFactor);

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