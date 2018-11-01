#include <tuple>
#include <string>

using namespace std;

class TivaController 
{
	typedef tuple<int,int> Point;
	public:
		// getter methods
		double getMotor1Angle() const {return q1;}
		double getMotor2Angle() const {return q2;}
		Point getArm1Location() const {return make_tuple(x1, y1);}
		Point getArm2Location() const {return make_tuple(x2, y2);}
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
		tuple<double,double> computeKinematics(double x, double y, bool negative);

		// Constructor
		TivaController(double _unitsPerCm, double arm1Cm, double arm2Cm, 
					   double xOffsetCm, double yOffsetCm);
	
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