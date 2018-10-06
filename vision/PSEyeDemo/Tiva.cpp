#include <iostream>     // std::cout
#include <tuple>        // std::tuple, std::get, std::tie, std::ignore
#include <string>
#include <cmath>

using namespace std;

class TivaController 
{
	public:
		// getter methods
		double getMotor1Angle() const {return q1;}
		double getMotor2Angle() const {return q2;}
		tuple<double, double> getArm1Location() const {return make_tuple(x1, y1);}
		tuple<double, double> getArm2Location() const {return make_tuple(x2, y2);}
		double getArm1Length() const {return a1;}
		double getArm2Length() const {return a2;}
		double getxOffset() const {return xOffset;}
		double getyOffset() const {return yOffset;}

		// setter methods
		void setMotor1Angle(double new_q1) { q1 = new_q1; updateArmLocation();}
		void setMotor2Angle(double new_q2) { q2 = new_q2; updateArmLocation();}
		void setXOffsetCm(double new_xOffsetCm) {xOffset = new_xOffsetCm * unitsPerCm;}
		void setYOffsetCm(double new_yOffsetCm) {xOffset = new_yOffsetCm * unitsPerCm;}
		void setArm1Cm(double new_arm1Cm) {a1 = new_arm1Cm * unitsPerCm;}
		void setArm2Cm(double new_arm2Cm) {a2= new_arm2Cm * unitsPerCm;}

		// arm movement methods
		void moveArm(double x, double y, bool negative);
		void resetArm();
		void updateArmLocation();
		tuple<double, double> computeKinematics(double x, double y, bool negative);

		// Constructor
		TivaController(double _unitsPerCm, double arm1Cm, double arm2Cm, 
					   double xOffsetCm, double yOffsetCm)
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
	
	private:
		double q1;
		double q2;
		double x1;
		double y1;
		double x2;
		double y2;

		double unitsPerCm;
		double xOffset;
		double yOffset;
		double a1;
		double a2;
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

int main() {
	TivaController Tiva = TivaController(1.0, 45.0, 20.0, 33.0, -10.0);
	cout << Tiva.getMotor1Angle() << "\n";
	Tiva.moveArm(5.0, 20.0, false);
	cout << Tiva.getMotor1Angle() << "\n";
}