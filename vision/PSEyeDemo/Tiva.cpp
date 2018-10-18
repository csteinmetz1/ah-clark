#include "Tiva.h"

// getter methods
double TivaController::getMotor1Angle() const {return q1;}
double TivaController::getMotor2Angle() const {return q2;}
Vec TivaController::getArm1Location() const {return arm1Pos;}
Vec TivaController::getArm2Location() const {return arm2Pos;}
double TivaController::getArm1Length() const {return a1;}
double TivaController::getArm2Length() const {return a2;}
double TivaController::getxOffset() const {return xOffset;}
double TivaController::getyOffset() const {return yOffset;}

// setter methods
void TivaController::setMotor1Angle(double new_q1) { q1 = new_q1; updateArmLocation();}
void TivaController::setMotor2Angle(double new_q2) { q2 = new_q2; updateArmLocation();}
void TivaController::setXOffsetCm(double new_xOffsetCm) {xOffset = new_xOffsetCm * unitsPerCm;}
void TivaController::setYOffsetCm(double new_yOffsetCm) {xOffset = new_yOffsetCm * unitsPerCm;}
void TivaController::setArm1Cm(double new_arm1Cm) {a1 = new_arm1Cm * unitsPerCm;}
void TivaController::setArm2Cm(double new_arm2Cm) {a2 = new_arm2Cm * unitsPerCm;}

TivaController::TivaController(double _unitsPerCm, double arm1Cm, double arm2Cm, double xOffsetCm, double yOffsetCm)
{
	unitsPerCm = _unitsPerCm;
	xOffset = unitsPerCm * xOffsetCm;
	yOffset = unitsPerCm * yOffsetCm;
	a1 = unitsPerCm * arm1Cm;
	a2 = unitsPerCm * arm2Cm;

	// initialize arm 

	Vec arm1Pos;
	Vec arm2Pos;

	q1 = 0.0;
	q2 = 0.0;
	//x1 = 0.0;
	//y1 = 0.0;
	//x2 = 0.0;
	//y2 = 0.0;
};

void TivaController::resetArm(void) 
{
	setMotor1Angle(0.0);
	setMotor2Angle(0.0);
	updateArmLocation();
};

void TivaController::moveArm(Vec point, bool negative) 
{
	std::tuple<double,double> newAngles;
	newAngles = computeKinematics(point, negative);
	setMotor1Angle(std::get<0>(newAngles));
	setMotor2Angle(std::get<1>(newAngles));
	updateArmLocation();
};

std::tuple<double,double> TivaController::computeKinematics(Vec point, bool negative) 
{ 
	double new_q1, new_q2;

	point.x -= xOffset;
	point.y -= yOffset;

	if (negative) // negative q2 solution
	{
		new_q2 = -(acos(((pow(point.x,2) + pow(point.y,2) - pow(a1,2) - pow(a2,2)) / (2.0 * a1 * a2))));
		new_q1 = atan2(point.y, point.x) - atan2((a2 * sin(new_q2)), (a1 + (a2 * cos(new_q2))));
	}
	else 		 // positive q2 solution
	{
		new_q2 =  acos( (pow(point.x,2) + pow(point.y,2) - pow(a1,2) - pow(a2,2)) / (2.0 * a1 * a2) );
		new_q1 = atan2(point.y, point.x) - atan2((a2 * sin(new_q2)), (a1 + (a2 * cos(new_q2))));
	}
	return std::make_tuple(new_q1, new_q2);
}

void TivaController::updateArmLocation() 
{
	arm1Pos.x = a1 * cos(q1) + xOffset;
	arm1Pos.y = a1 * sin(q1) + yOffset;
	arm2Pos.x = arm1Pos.x + (a2 * cos(q1+q2));
	arm2Pos.y = arm1Pos.y + (a2 * sin(q1+q2));
}


int main() {

	// instaniate Tiva object
	TivaController Tiva = TivaController(1.0, 45.0, 20.0, 33.0, -10.0);

	// instantiate set point for arm
	Vec setPoint;
	setPoint.x = 50.0;
	setPoint.y = 20.0;

	// move arm and show new angle
	std::cout << Tiva.getMotor1Angle() << "\n";
	Tiva.moveArm(setPoint, false);
	std::cout << Tiva.getMotor1Angle() << "\n";

	// instantiate puck
	Vec initPos;
	Vec initVel;
	Vec initAcl;

	initPos.x = 30.0;
	initPos.y = 60.0;

	initVel.x = -0.5;
	initVel.y = -2.0;

	initAcl.x = -0.0125;
	initAcl.y = -0.0125;

	double radius = 50.0;
	double widthCm = 66.0;
	double heightCm = 136.0;
	Puck puck = Puck(initPos, initVel, initAcl, radius, 1.0, widthCm, heightCm);

	// vector to hold trajectory points
	std::vector<Vec> trajectory;

	int estimation_size = 60;

	trajectory = puck.computeTrajectory(puck, estimation_size);

	for (auto current_pos : trajectory)
	{
    	std::cout << "x: " << current_pos.x << " y: " << current_pos.y << std::endl;
	}

	return 0;

}