#include <iostream>     // std::cout
#include <tuple>        // std::tuple, std::get, std::tie, std::ignore
#include <string>
#include <cmath>

using namespace std;

class TivaController 
{
	public:

		double getMotor1Angle() const {return q1;}
		double getMotor2Angle() const {return q2;}
		tuple<double, double> getArm1Location() const {return make_tuple(x1, y1);}
		tuple<double, double> getArm2Location() const {return make_tuple(x2, y2);}
		double getArm1Length() const {return a1;}
		double getArm2Length() const {return a2;}
		double getxOffset() const {return xOffset;}
		double getyOffset() const {return yOffset;}


		void setMotor1Angle(double new_q1) { q1 = new_q1; updateArmLocation();}
		void setMotor2Angle(double new_q2) { q2 = new_q2; updateArmLocation();}
		// excluding setter for offsets and arm lengths at the moment
		// user cannot control arm by adjusting x1,y1 x2,y2 points


		void computeKinematics(bool negative) 
		{ 
			// negative q2 solution
        	double q2_neg = -(acos((x**2 + y**2 - self.a1**2 - self.a2**2) / (2.0 * self.a1 * self.a2)))
        	double q1_neg = np.atan2(y, x) - np.atan2( (self.a2 * np.sin(q2_neg)), (self.a1 + (self.a2 * np.cos(q2_neg))) )

			// postive q2 solution
			double q2_pos = np.arccos((x**2 + y**2 - self.a1**2 - self.a2**2) / (2.0 * self.a1 * self.a2))
			double q1_pos = np.arctan2(y, x) - np.arctan2( (self.a2 * np.sin(q2_pos)), (self.a1 + (self.a2 * np.cos(q2_pos))) )
		}

		void updateArmLocation() 
		{
			x1 = a1 * cos(q1) + xOffset;
			y1 = a1 * sin(q1) + yOffset;
			x2 = x1 + (a2 * cos(q1+q2));
			y2 = y1 + (a2 * sin(q1+q2));
		}

		void resetArm() { setMotor1Angle(0.0); setMotor2Angle(0.0);};
		void moveArm(double x, double y);


		// Constructor
		TivaController(double unitsPerCm, double arm1Cm, double arm2Cm, 
					   double xOffsetCm, double yOffsetCm)
		{
			xOffset = unitsPerCm * xOffsetCm;
			yOffset = unitsPerCm * yOffsetCm;
			a1 = unitsPerCm * arm1Cm;
			a2 = unitsPerCm * arm2Cm;
		};
	
	private:
		double q1;
		double q2;
		double x1;
		double y1;
		double x2;
		double y2;

		double xOffset;
		double yOffset;
		double a1;
		double a2;
};

void TivaController::resetArm(void) {
	//arm_position.q1 = 0.0;
	//arm_position.q2 = 0.0;
};

void TivaController::moveArm(double x, double y) {

	//double x_rel -= x_offset_cm;

};


int main() {
	TivaController Tiva = TivaController(1, 45.0, 29.0, 0.0, 0.0);
	cout << Tiva.getMotor1Angle() << "\n";
}