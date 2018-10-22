#include "Tiva.h"

// getter methods
double TivaController::getMotor1Angle() const {return q1;}
double TivaController::getMotor2Angle() const {return q2;}
Comp TivaController::getArm1Location() const {return arm1Pos;}
Comp TivaController::getArm2Location() const {return arm2Pos;}
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

	Comp arm1Pos;
	Comp arm2Pos;

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

void TivaController::moveArm(Comp point, bool negative) 
{
	std::tuple<double,double> newAngles;
	newAngles = computeKinematics(point, negative);
	setMotor1Angle(std::get<0>(newAngles));
	setMotor2Angle(std::get<1>(newAngles));
	updateArmLocation();
};

std::tuple<double,double> TivaController::computeKinematics(Comp point, bool negative) 
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

std::vector<Comp> TivaController::computePath(Comp start, Comp stop, int steps)
{
	std::vector<Comp> path;
	Comp point;

	for (int i = 0; i <= steps; i++)
	{
		point.x = start.x + i * ( (stop.x - start.x) / steps);
		point.y = start.y + i * ( (stop.y - start.y) / steps);
		path.push_back(point);
	}

	return path;
}

int main() {

	// instaniate Tiva object
	TivaController Tiva = TivaController(1.0, 45.0, 20.0, 33.0, -10.0);

	// instantiate set point for arm
	Comp setPoint;
	setPoint.x = 50.0;
	setPoint.y = 20.0;

	// move arm and show new angle
	//std::cout << Tiva.getMotor1Angle() << "\n";
	Tiva.moveArm(setPoint, false);
	//std::cout << Tiva.getMotor1Angle() << "\n";

	// instantiate puck
	Comp initPos;
	Comp secondPos;
	Comp initAcl;

	initPos.x = 33.0;
	initPos.y = 100.0;

	secondPos.x = 30.0;
	secondPos.y = 85.0;

	int frames = 5;

	initAcl.x = 0;
	initAcl.y = 0;

	double radius = 0.0;
	double widthCm = 66.0;
	double heightCm = 136.0;
	Puck puck = Puck(initPos, secondPos, initAcl, radius, 1.0, widthCm, heightCm, frames);

	// vector to hold trajectory points
	std::vector<Comp> trajectory;

	// vector to hold path points
	std::vector<Comp> path;

	int estimation_size = 60;

	trajectory = puck.computeTrajectory(estimation_size);

	for (auto current_pos : trajectory)
	{
		if (current_pos.y < 35) {
    		std::cout << "Target x: " << current_pos.x << " y: " << current_pos.y << std::endl;
			path = Tiva.computePath(Tiva.getArm2Location(), current_pos, 10);
			break;
		}
	}

	for (auto point : path) 
	{
		std::cout << "x: " << point.x << " y: " << point.y << std::endl;
	}

	return 0;

}