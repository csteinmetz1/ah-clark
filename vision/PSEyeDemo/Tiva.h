#include <tuple>
#include <string>
#include <iostream> 
#include <cmath>
#include "Puck.h"

class TivaController 
{
	public:
		// getter methods
		double getMotor1Angle() const;
		double getMotor2Angle() const;
		Comp getArm1Location() const;
		Comp getArm2Location() const;
		double getArm1Length() const;
		double getArm2Length() const;
		double getxOffset() const;
		double getyOffset() const;

		// setter methods
		void setMotor1Angle(double new_q1);
		void setMotor2Angle(double new_q2);
		void setXOffsetCm(double new_xOffsetCm);
		void setYOffsetCm(double new_yOffsetCm);
		void setArm1Cm(double new_arm1Cm);
		void setArm2Cm(double new_arm2Cm);

		// arm movement methods
		void resetArm();
		void updateArmLocation();
		void moveArm(Comp point, bool negative);
		static std::vector<Comp> computePath(Comp start, Comp end, int steps);
		std::tuple<double,double> computeKinematics(Comp point, bool negative);

		// Constructor
		TivaController(double _unitsPerCm, double arm1Cm, double arm2Cm, 
					   double xOffsetCm, double yOffsetCm);
	
	private:
		double q1;
		double q2;
		Comp arm1Pos;
		Comp arm2Pos;
		
		//double x1;
		//double y1;
		//double x2;
		//double y2;

		double unitsPerCm;
		double xOffset;
		double yOffset;
		double a1;
		double a2;
};