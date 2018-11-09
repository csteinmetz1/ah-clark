#include "stdafx.h"
#include "Tiva.h"

// getter methods
double TivaController::getMotor1AngleRadians() const {return q1;}
double TivaController::getMotor2AngleRadians() const {return q2;}
double TivaController::getMotor1AngleDegrees() const {return q1 * (180.0 / 3.141592653589793238463);}
double TivaController::getMotor2AngleDegrees() const {return q2 * (180.0 / 3.141592653589793238463);}
Vec_double TivaController::getArm1Location() const {return arm1Pos;}
Vec_double TivaController::getArm2Location() const {return arm2Pos;}
double TivaController::getArm1Length() const {return a1;}
double TivaController::getArm2Length() const {return a2;}
double TivaController::getxOffset() const {return xOffset;}
double TivaController::getyOffset() const {return yOffset;}

// setter methods
void TivaController::setMotor1AngleRadians(double new_q1) { q1 = new_q1; updateArmLocation();}
void TivaController::setMotor2AngleRadians(double new_q2) { q2 = new_q2; updateArmLocation();}
void TivaController::setMotor1AngleDegrees(double new_q1) { q1 = new_q1 * (3.141592653589793238463 / 180.0); updateArmLocation();}
void TivaController::setMotor2AngleDegrees(double new_q2) { q2 = new_q2 * (3.141592653589793238463 / 180.0); updateArmLocation();}
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
	resetArm();
};

void TivaController::resetArm(void)
{
	setMotor1AngleRadians(0.0);
	setMotor2AngleRadians(0.0);
	updateArmLocation();
};


void TivaController::moveArm(Vec_double point, bool negative) 
{
	std::tuple<double,double> newAngles;
	newAngles = computeKinematics(point, negative);
	setMotor1AngleRadians(std::get<0>(newAngles));
	setMotor2AngleRadians(std::get<1>(newAngles));
	updateArmLocation();
};

std::tuple<double,double> TivaController::computeKinematics(Vec_double point, bool negative) 
{ 
	double new_q1, new_q2;

	point.x -= xOffset;
	point.y -= yOffset;

	if (negative) // negative q2 solution
	{
		new_q2 = -(acos(((pow(point.x, 2) + pow(point.y, 2) - pow(a1, 2) - pow(a2, 2)) / (2.0 * a1 * a2))));
		new_q1 =   atan2(point.y, point.x) - atan2((a2 * sin(new_q2)), (a1 + (a2 * cos(new_q2))));
	}
	else 		 // positive q2 solution
	{
		new_q2 = acos((pow(point.x, 2) + pow(point.y, 2) - pow(a1, 2) - pow(a2, 2)) / (2.0 * a1 * a2));
		new_q1 = atan2(point.y, point.x) - atan2((a2 * sin(new_q2)), (a1 + (a2 * cos(new_q2))));
	}
  // check if solution is valid arm location
	if (isnan(new_q1) || isnan(new_q2)) {
		new_q1 = q1;
		new_q2 = q2;
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

std::vector<Vec_double> TivaController::computePath(Vec_double start, Vec_double stop, int steps=-1)
{
	std::vector<Vec_double> path; // vector of points on linear path
	Vec_double point;				      // point struct
	double distance;		          // distance of path

	// if no step size is given calculate based on distance
	if (steps == -1) { 
		// compute distance to new point
		distance = sqrt(pow(stop.x - start.x, 2) + pow(stop.y - start.y, 2));
		steps = (floor(distance) + 1) * 4;
	}
	
	for (int i = 0; i <= steps; i++)
	{
		point.x = start.x + i * ( (stop.x - start.x) / steps);
		point.y = start.y + i * ( (stop.y - start.y) / steps);
		path.push_back(point);
  }
	return path;
}

//////////////////////////////////////////////////////////
/* this is an example of how to use the class

int main() {

	// instaniate Tiva object
	TivaController Tiva = TivaController(1.0, 45.0, 20.0, 33.0, -10.0);

	// instantiate set point for arm
	Vec_double setPoint;

	setPoint.x = 25;
	setPoint.y = 30;

	// move arm and show new angle
	//std::cout << Tiva.getMotor1Angle() << "\n";
	Tiva.moveArm(setPoint, false);
	//std::cout << Tiva.getMotor1Angle() << "\n";

	// instantiate puck
	Vec_double initPos;
	Vec_double secondPos;
	Vec_double initAcl;

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
	std::vector<Vec_double> trajectory;

// vector to hold path points
	std::vector<Vec_double> path;

	int estimation_size = 60;

	trajectory = puck.computeTrajectory(estimation_size);

	for (auto current_pos : trajectory)
	{
		if (current_pos.y < 35) {
    		std::cout << "Target x: " << current_pos.x << " y: " << current_pos.y << std::endl;
			path = Tiva.computePath(Tiva.getArm2Location(), current_pos);
			break;
		}
	}

	for (auto point : path) 
	{
		std::cout << "x: " << point.x << " y: " << point.y << std::endl;
	}

	return 0;
}
*/

