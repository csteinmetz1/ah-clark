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
	std::vector<Vec_double> path; 		// vector of points on linear path
	Vec_double point;				    // point struct
	double distance;		          	// distance of path

	// if no step size is given calculate based on distance
	if (steps == -1) { 
		// compute distance to new point
		distance = sqrt(pow(stop.x - start.x, 2) + pow(stop.y - start.y, 2));
		steps = 2 * (floor(distance) + 1);
	}
	
	for (int i = 0; i <= steps; i++)
	{
		point.x = start.x + i * ( (stop.x - start.x) / steps);
		point.y = start.y + i * ( (stop.y - start.y) / steps);
		path.push_back(point);
  	}
	return path;
}

std::vector<Vec_double> TivaController::computeHitPath(std::vector<Vec_double> trajectory, Vec_double targetPoint, 
													  double fps, double yhit, double xlim, double ylim, int minSteps)
{
	Vec_double hitPoint;
	hitPoint.x = -1; // default value for flag
	hitPoint.y = -1;
	int hitFrame;

	for (int index = 0; index < trajectory.size(); ++index)
	{
		if (trajectory[index].y < yhit) 
		{
			hitPoint.x = trajectory[index].x;
			hitPoint.y = trajectory[index].y;
			hitFrame = index + 1; 	// find frame at which to hit the puck
			break;
		}
	}

	// if no hit point found return empty vector
	if (hitPoint.x == -1 && hitPoint.y == -1) 
	{
		std::vector<Vec_double> emptyPath;
		return emptyPath;
	}

	double slope; 			  // slope of the line connecting puck hit point and puck target point
	Vec_double hitStartPoint; // start point of the hit path which is behind the puck aimed at the target
	Vec_double hitEndPoint;   // end point of the hit path which is past the puck on the line towards the target

	slope = (targetPoint.y - hitPoint.y)  / (targetPoint.x - hitPoint.x);

	//std::cout << slope << std::endl;

	if (slope < 0) // negative slope 
	{
		hitStartPoint.x = 66.0 - xlim;
		hitStartPoint.y = slope * (66.0 - xlim - hitPoint.x) + hitPoint.y;

		if (hitStartPoint.y < ylim) // check if y value is valid
		{
			hitStartPoint.y = ylim;
			hitStartPoint.x = (ylim - hitPoint.y) + hitPoint.x;
		}
	}
	else  // positive slope
	{
		hitStartPoint.x = xlim;
		hitStartPoint.y = slope * (xlim - hitPoint.x) + hitPoint.y;

		if (hitStartPoint.y < ylim) // check if y value is valid
		{
			hitStartPoint.y = ylim;
			hitStartPoint.x = ( (ylim - hitPoint.y) / slope ) + hitPoint.x;
		}
	}

	hitEndPoint.x = hitPoint.x - (hitPoint.x - targetPoint.x) * 0.15; 
	hitEndPoint.y = slope * (hitEndPoint.x - hitPoint.x) + hitPoint.y;	

	std::vector<Vec_double> initPath; // path from the current arm position to the strat of the hit path
	std::vector<Vec_double> hitPath;  // path from start point behind puck to point past the puck towards target
	std::vector<Vec_double> fullPath; // concatenation of the above two paths 

	double arrivalTime; // puck arrivial time in milliseconds
	int steps; 			// path steps to take to reach end point

	// compute number of steps to take (we assume one step takes ~2ms)
	arrivalTime = (hitFrame / fps) * 1000.0;
	steps = int(arrivalTime / 2.0) - 25;

	// checking for step size to enforce minimum step size
	if (steps < minSteps) {
		steps = minSteps;
	}

	hitPath = computePath(hitStartPoint, hitEndPoint, steps/2);
	initPath = computePath(arm2Pos, hitStartPoint, steps/2);

	//std::cout << hitPath.size() << initPath.size() << std::endl;

	// pack the two paths into a sigle vector
	fullPath.reserve( initPath.size() + hitPath.size() ); // preallocate memory
	fullPath.insert( fullPath.end(), initPath.begin(), initPath.end() );
	fullPath.insert( fullPath.end(), hitPath.begin(), hitPath.end() );

	return fullPath;
}

//////////////////////////////////////////////////////////
// this is an example of how to use the class 
/*
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
	Vec_double thirdPos;
	Vec_double initAcl;

	initPos.x = 33.0;
	initPos.y = 100.0;

	secondPos.x = 30.0;
	secondPos.y = 85.0;

	thirdPos.x = 27.5;
	thirdPos.y = 70.0;

	std::vector<Vec_double> points;
	points.push_back(initPos);
	points.push_back(secondPos);
	points.push_back(thirdPos);

	initAcl.x = 0;
	initAcl.y = 0;

	double radius = 0.0;
	double widthCm = 66.0;
	double heightCm = 136.0;
	Puck puck = Puck(points, initAcl, radius, 1.0, widthCm, heightCm);

	std::cout << puck.getVelocity().x << " " << puck.getVelocity().y << std::endl;

	// vector to hold trajectory points
	std::vector<Vec_double> trajectory;

	// vector to hold path points
	std::vector<Vec_double> path;
	trajectory = puck.computeTrajectory(60);

	std::cout << "trajectory" << std::endl;

	for (auto point : trajectory) {
		//std::cout << point.x << " " << point.y << std::endl;
	}

	// Now let's hit a puck
	std::vector<Vec_double> hitPath;

	// Define target - the center of the goal
	Vec_double targetPoint;
	targetPoint.x = 33.0;
	targetPoint.y = 136.0;

	hitPath = Tiva.computeHitPath(trajectory, targetPoint, 30.0);

	std::cout << "hit path" << std::endl;

	for (auto point : hitPath) {
		std::cout << point.x << " " << point.y << std::endl;
	}

	return 0;
}
*/

