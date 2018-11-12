#include "stdafx.h"
#include <string>
#include <stdexcept>
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

		// check if the center of the paddle will cause it to collide with the wall
		if      (point.x >= 61.0)
		{
			point.x = 61.0;
		}
		else if (point.x <= 5.0) 
		{	
			point.x = 5.0;
		}
		else if (point.y <= 5.0)
		{
			point.y = 5.0;
		}
		path.push_back(point);
  	}
	return path;
}

// --------------------------------------------------------------------------------------
// Valid hitTypes
// --------------------------------------------------------------------------------------
// "block"     - move the paddle to location of the puck at yhit    	(single motion)
// "hit"       - block the puck but continue moving past the puck 	    (single motion)
// "block+hit" - block the puck then quickly hit puck towards target    (two motions)
// "swing"     - swing at the puck hitting it toward a target 			(two motions)
// --------------------------------------------------------------------------------------
std::vector<Vec_double> TivaController::computeHitPath(std::vector<Vec_double> trajectory, 
Vec_double targetPoint, double fps, double yhit, double xlim, double ylim, int minSteps, std::string hitType)
{
	std::vector<Vec_double> emptyPath; // return this whenever there is no valid path
	Vec_double hitPoint; 	// placeholder for hit point intersection
	hitPoint.x = -1; 		// default value for flag to check if intersection is found
	hitPoint.y = -1;		
	int hitFrame;			// frame number of the puck hit point

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
		return emptyPath;
	}

	double arrivalTime; // puck arrivial time in milliseconds
	double timeOffset;  // how much earlier the paddle arrive in milliseconds
	int stepOffset;     // how much earlier the paddle arrives in steps
	int steps; 			// path steps to take to reach end point

	timeOffset = 100.0; 					// ms
	stepOffset = int(timeOffset / 2.0); // steps

	// compute number of steps to take (we assume one step takes ~2ms)
	arrivalTime = (hitFrame / fps) * 1000.0;
	steps = int(arrivalTime / 2.0) - stepOffset;

	// realize that if steps less than min steps, 
	// arm will not reach location in time to hit the puck!
	if (steps < minSteps) {
		// idea - maybe move arm to a goal block location if it can't hit it?
		return emptyPath; // jump out if won't reach in time
	}

	// Compute paths based on the set mode assuming enough steps
	if      (hitType == "block")
	{
		std::vector<Vec_double> blockPath;
		blockPath = computePath(arm2Pos, hitPoint, steps); // compute direct path to puck

		return blockPath;
	}
	else if (hitType == "hit")
	{
		double slope; 			// slope of the line connecting puck hit point and puck target point
		Vec_double hitEndPoint; // end point of the hit path which is past the puck on the line towards the target

		slope = (hitPoint.y - arm2Pos.y)  / (hitPoint.x - arm2Pos.x);

		hitEndPoint.y = 35.0;
		hitEndPoint.x = ( (hitEndPoint.y - hitPoint.y) / slope ) + hitPoint.x;

		std::vector<Vec_double> blockPath; // path from the current arm position to the puck intercept
		std::vector<Vec_double> hitPath;   // continue through the puck a number of steps
		std::vector<Vec_double> fullPath;  // concatenation of the above two paths 

		blockPath = computePath(arm2Pos, hitPoint, steps);   // compute path to puck to "block"
		hitPath   = computePath(hitPoint, hitEndPoint, 100); // compute path to hit through puck 

		fullPath.reserve( blockPath.size() + hitPath.size() ); // preallocate memory
		fullPath.insert( fullPath.end(), blockPath.begin(), blockPath.end() );
		fullPath.insert( fullPath.end(), hitPath.begin(), hitPath.end() );

		return fullPath;
	}
	else if (hitType == "block+hit")
	{
		double slope; 			  // slope of the line connecting puck hit point and puck target point
		Vec_double hitEndPoint;   // end point of the hit path which is past the puck on the line towards the target

		slope = (targetPoint.y - hitPoint.y)  / (targetPoint.x - hitPoint.x);

		hitEndPoint.y = 35.0;
		hitEndPoint.x = ( (hitEndPoint.y - hitPoint.y) / slope ) + hitPoint.x;

		std::vector<Vec_double> blockPath; // path from the current arm position to the puck intercept
		std::vector<Vec_double> hitPath;   // path from the block to point past the puck towards target
		std::vector<Vec_double> fullPath;  // concatenation of the above two paths 

		blockPath = computePath(arm2Pos, hitPoint, int((3.0/4.0)*steps));	 // compute path to puck
		hitPath   = computePath(hitPoint, hitEndPoint, int((1.0/4.0)*steps)); // quickly hit puck towards target

		// pack the two paths into a sigle vector
		fullPath.reserve( blockPath.size() + hitPath.size() ); // preallocate memory
		fullPath.insert( fullPath.end(), blockPath.begin(), blockPath.end() );
		fullPath.insert( fullPath.end(), hitPath.begin(), hitPath.end() );

		return fullPath;
	}
	else if (hitType == "swing")
	{
		double slope; 			  // slope of the line connecting puck hit point and puck target point
		Vec_double hitStartPoint; // start point of the hit path which is behind the puck aimed at the target
		Vec_double hitEndPoint;   // end point of the hit path which is past the puck on the line towards the target

		slope = (targetPoint.y - hitPoint.y)  / (targetPoint.x - hitPoint.x);

		if (slope < 0) // negative slope 
		{
			hitStartPoint.x = 66.0 - xlim;
			hitStartPoint.y = slope * (66.0 - xlim - hitPoint.x) + hitPoint.y;

			if (hitStartPoint.y < ylim) // check if y value is valid
			{
				hitStartPoint.y = ylim;
				hitStartPoint.x = ( ylim - hitPoint.y / slope ) + hitPoint.x;
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

		hitPath  = computePath(hitStartPoint, hitEndPoint, steps/2);
		initPath = computePath(arm2Pos, hitStartPoint, steps/2);

		// pack the two paths into a sigle vector
		fullPath.reserve( initPath.size() + hitPath.size() ); // preallocate memory
		fullPath.insert( fullPath.end(), initPath.begin(), initPath.end() );
		fullPath.insert( fullPath.end(), hitPath.begin(), hitPath.end() );

		return fullPath;
	}
	else
	{
		throw std::invalid_argument("invalid hit type - must be 'block', 'hit, 'block+hit', or 'swing'.");
	}
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

	initPos.x = 30.0;
	initPos.y = 120.0;

	secondPos.x = 29.0;
	secondPos.y = 115.0;

	thirdPos.x = 28.0;
	thirdPos.y = 110.0;

	std::vector<Vec_double> points;
	points.push_back(initPos);
	points.push_back(secondPos);
	points.push_back(thirdPos);

	initAcl.x = 0;
	initAcl.y = 0;

	double radius = 3.5;
	double widthCm = 66.0;
	double heightCm = 136.0;
	Puck puck = Puck(points, initAcl, radius, 1.0, widthCm, heightCm);

	std::cout << "vel: " << puck.getVelocity().x << " " << puck.getVelocity().y << std::endl;

	// Now let's hit a puck
	std::vector<Vec_double> hitPath;

	// Define target - the center of the goal
	Vec_double targetPoint;
	targetPoint.x = 33.0;
	targetPoint.y = 136.0;

	for ( auto point : puck.getTrajectory() )
	{
		//std::cout << point.x << " " << point.y << std::endl;
	}

	hitPath = Tiva.computeHitPath(puck.getTrajectory(), targetPoint, 30.0, 25, 10, 10, 100, "hit");

	if (hitPath.size() > 0 )
	{
		std::cout << "hit path" << std::endl;
		for (auto point : hitPath) {
			std::cout << point.x << " " << point.y << std::endl;
		}
	}
	else 
	{
		std::cout << "no valid hit path" << std::endl;
	}
	return 0;
}
*/

