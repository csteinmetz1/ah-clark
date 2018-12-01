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

std::vector<Vec_double> TivaController::computeLinearPath(Vec_double start, Vec_double stop, int steps, bool safety=true)
{
	std::vector<Vec_double> path; 		// vector of points on linear path
	Vec_double point;				    // point struct
	double distance;		          	// distance of pathc
	double distancePerStep;				// distance to be travelled during each step

	// compute distance to new point
	distance = sqrt(pow(stop.x - start.x, 2) + pow(stop.y - start.y, 2));

	if (steps == 0)
	{
		steps = int(12.4 * (floor(distance) + 1));
	}

	distancePerStep = distance / double(steps);

	// NOTE: maxDistancePerStep is a parameter we need to determine by some testing with set PID values
	double maxDistancePerStep = 0.20; // this was .20 but we need to try it lower

	std::cout << "distance per step: " << distancePerStep << std::endl;
	//std::cout << "steps: " << steps << std::endl;
	//std::cout << "distance: " << distance << std::endl;

	if (distancePerStep > maxDistancePerStep && safety) 
	{ 
		// return empty path if the distance per step is too large
		// this helps to tame erratic behaviour
		return path; 
	}
	
	for (int i = 0; i <= steps; i++)
	{
		point.x = start.x + i * ( (stop.x - start.x) / steps);
		point.y = start.y + i * ( (stop.y - start.y) / steps);
		path.push_back(point);
  	}
	return path;
}


std::vector<Vec_double> TivaController::computeCurvedPath(Vec_double start, Vec_double stop, int steps, double rFactor)
{
	std::vector<Vec_double> path; 		// vector of points on linear path
	Vec_double point;				    // point struct
	double arcLength;		          	// distance of path
	double distancePerStep;				// distance to be travelled during each step

	// compute the midpoint between start and stop points
	Vec_double midpoint;
	midpoint.x = (start.x + stop.x) / 2.0;
	midpoint.y = (start.y + stop.y) / 2.0;

	//std::cout << "midpoint x: " << midpoint.x << " y:" << midpoint.y << std::endl;

	// compute line between start and stop points
	double chord_slope = (stop.y - start.y)  / (stop.x - start.x);

	// compute perpendicular bisector slope
	double bisect_slope = -(1.0 / chord_slope);

	//std::cout << "chord slope: " << chord_slope << " bisect slope: " << bisect_slope << std::endl;

	// compute center of the fitted circle (this can have two solutions)
	Vec_double center;

	if (stop.x > start.x ) // positive angle direction 
	{
		center.y = midpoint.y - (bisect_slope * rFactor);
		center.x = midpoint.x - (1.0 * rFactor); 
	}
	else if (stop.x < start.x) // negative angle direction 
	{
		center.y = midpoint.y + (bisect_slope * rFactor); 
		center.x = midpoint.x + (1.0 * rFactor); 
	}

	//std::cout << "center x: " << center.x << " y: " << center.y << std::endl;

	// find the radius of the circle (distance from center to start point)
	double radius = sqrt(pow(center.x - start.x, 2) + pow(center.y - start.y, 2));

	//std::cout << "radius: " << radius << std::endl;

	// find starting and ending angle 
	double thetaStart = atan2( (start.y - center.y), (start.x - center.x) );
	double thetaStop  = atan2( (stop.y - center.y), (stop.x - center.x) );

	// turn all negative angles into positive angles
	if ( thetaStart < 0 )
	{
		thetaStart += 2 * M_PI;
	}
	if (thetaStop < 0 )
	{
		thetaStop += 2 * M_PI;
	}

	//std::cout << "thetaStart: "<< thetaStart << " thetaStop: " << thetaStop << std::endl;

	// compute the distance per step based on arc length of path
	arcLength = 2 * M_PI * radius * (abs(thetaStart - thetaStop));
	distancePerStep = arcLength / double(steps);

	std::cout << "distance per step: " << distancePerStep << std::endl;

	// finally compute the positive or negative direction path
	double theta = thetaStart;
	double thetaDistance;;

	//std::cout << "thetaDistance: " << thetaDistance << std::endl;

	if (stop.x > start.x) // positive direction 
	{
		thetaDistance = (2 * M_PI - thetaStart) + thetaStop;

		while (theta <= thetaStart + thetaDistance) 
		{
			point.x = radius * cos(theta) + center.x;
			point.y = radius * sin(theta) + center.y ;
			path.push_back(point);
			theta += ( (thetaDistance) / steps );
		}
	}
	else if (stop.x < start.x) // negative direction
	{
		thetaDistance = abs(thetaStop - thetaStart);
		while (theta >= thetaStop)
		{
			point.x = radius * cos(theta) + center.x;
			point.y = radius * sin(theta) + center.y ;
			path.push_back(point);
			theta -= ( (thetaDistance) / steps );
		}
	}
	return path;
}


std::tuple<Vec_double, int> TivaController::findBlockPoint(std::vector<Vec_double> trajectory, double yblock)
{
	Vec_double blockPoint;
	int blockFrame = 0;

	blockPoint.x = -1;
	blockPoint.y = -1;

	std::string blockType = "line";

	if (blockType == "triangle")
	{
		double leftLineSlope = (10.0 - 3.5) / (33.0 - 15.0);
		double rightLineSlope = (10.0 - 3.5) / (33.0 - 48.0);

		for (int index = 0; index < trajectory.size(); ++index)
		{
			if (trajectory[index].y < (leftLineSlope * (trajectory[index].x - 33.0) + 10.0) && (trajectory[index].x < 33.0))
			{
				blockPoint.x = trajectory[index].x;
				blockPoint.y = trajectory[index].y;
				blockFrame = index + 1; 	// find frame at which to hit the puck
				break;
			}
			if (trajectory[index].y < (rightLineSlope * (trajectory[index].x - 33.0) + 10.0) && (trajectory[index].x > 33.0))
			{
				blockPoint.x = trajectory[index].x;
				blockPoint.y = trajectory[index].y;
				blockFrame = index + 1; 	// find frame at which to hit the puck
				break;
			}
		}
	}
	else if (blockType == "line")
	{
		for (int index = 0; index < trajectory.size(); ++index)
		{
			if (trajectory[index].y < yblock)
			{
				blockPoint.x = trajectory[index].x;
				blockPoint.y = trajectory[index].y;
				blockFrame = index + 1; 	// find frame at which to hit the puck
				break;
			}
		}
	}
	return std::make_tuple(blockPoint, blockFrame);
}

/*
+---------------------------------------------------------------------------------------+
| Puck Hitting Methods																    |
+---------------+-----------------------------------------------------+-----------------+
| Block         | move the paddle to location of the puck at yhit     | (single motion) |
| Hit           | block the puck but continue moving past the puck    | (single motion) |
| Block and Hit | block the puck then quickly hit puck towards target | (two motions)   |
| Swing         | swing at the puck hitting it toward a target        | (two motions)   |
+---------------+-----------------------------------------------------+-----------------+*/

std::vector<Vec_double> TivaController::computeBlockPath(std::vector<Vec_double> trajectory, double sampleTime, double yblock)
{

	std::tuple<Vec_double, int> block; // block tuple | 0 - block point | 1 - block frame
	Vec_double blockPoint;	  // point to intercept the puck and block it
	int blockFrame;			  // frame in the future at which the puck is to be intercepted

	block = findBlockPoint(trajectory, yblock);
	blockPoint = std::get<0>(block);
	blockFrame = std::get<1>(block);

	double arrivalTime = blockFrame * sampleTime * 1000.0; 	// est. time in milliseconds
	int steps = int(arrivalTime / 2.0);						// est. number of steps (we assume each step takes ~2ms)

	std::vector<Vec_double> blockPath;

	blockPath = computeLinearPath(arm2Pos, std::get<0>(block), steps); // compute direct path to puck

	return blockPath;

}

std::vector<Vec_double> TivaController::computeBlockAndHitPath(std::vector<Vec_double> trajectory, Vec_double targetPoint, double sampleTime, double yblock, double stepFactor)
{
	// Compute blocking details
	std::tuple<Vec_double, int> block; // block tuple | 0 - block point | 1 - block frame
	Vec_double blockPoint;	  // point to intercept the puck and block it
	int blockFrame;			  // frame in the future at which the puck is to be intercepted

	// generate paths
	std::vector<Vec_double> blockPath; // path from the current arm position to the puck intercept
	std::vector<Vec_double> hitPath;   // path from the block to point past the puck towards target
	std::vector<Vec_double> fullPath;  // concatenation of the above two paths 

	block = findBlockPoint(trajectory, yblock);

	blockPoint = std::get<0>(block);
	blockFrame = std::get<1>(block);

	// return empty path if no intersection point is found
	if (blockPoint.x == -1 || blockPoint.y == -1)
	{
		return fullPath;
	}

	// compute hitting details
	double hittingSlope; 	// slope of the line connecting puck hit point and puck target point
	double trajectorySlope; // slope of the incoming trajectory line
	Vec_double hitEndPoint; // end point of the hit path which is past the puck on the line towards the target

	trajectorySlope = (trajectory.end()[-2].y - blockPoint.y)  / (trajectory.end()[-2].x - blockPoint.x);

	// determine targetPoint to based on hit point to bank off wall
	if (abs(trajectorySlope) > 5.0)
	{
		targetPoint.x = 33.0;
		targetPoint.y = 136.0;
	}
	else if (trajectory.end()[-2].x < blockPoint.x)
	{
		targetPoint.x = 0.0;
		targetPoint.y = 75.0;
	}
	else
	{
		targetPoint.x = 66.0;
		targetPoint.y = 75.0;
	}

	hittingSlope = (targetPoint.y - blockPoint.y)  / (targetPoint.x - blockPoint.x);

	hitEndPoint.y = 35.0;
	hitEndPoint.x = ( (hitEndPoint.y - blockPoint.y) / hittingSlope ) + blockPoint.x;

	// calculate number of steps
	double offsetTime = 0.0;								// time to arrive early in ms
	double arrivalTime = blockFrame * sampleTime * 1000.0; 	// est. time in milliseconds
	int steps = int((arrivalTime - offsetTime) / 2.0);		// est. number of steps (we assume each step takes ~2ms)

	// check stepFactor if in valid range
	if (stepFactor <= 0 || stepFactor >= 1.0)
	{
		 throw std::invalid_argument("stepFactor must be between 0.0 and 1.0 - non-inclusive");
	}

	// if negative steps are calculated return an empty path
	if (steps <= 0)
	{
		return fullPath;
	}

	blockPath = computeLinearPath(arm2Pos, blockPoint, (stepFactor*steps), true);	 // compute path to puck
	hitPath   = computeLinearPath(blockPoint, hitEndPoint, (1.0-stepFactor)*steps, false); // quickly hit puck towards target

	// this will catch if any of the composite paths are empty
	if (blockPath.size() == 0)
	{
		return blockPath;
	}
	else if (hitPath.size() == 0) 
	{
		return hitPath;
	}

	// pack the two paths into a sigle vector
	fullPath.reserve( blockPath.size() + hitPath.size() ); // preallocate memory
	fullPath.insert( fullPath.end(), blockPath.begin(), blockPath.end() );
	fullPath.insert( fullPath.end(), hitPath.begin(), hitPath.end() );

	return fullPath;
}

std::vector<Vec_double> TivaController::computeSwingPath(std::vector<Vec_double> trajectory, Vec_double targetPoint, double sampleTime, double yblock, double stepFactor)
{
	// Compute blocking details
	std::tuple<Vec_double, int> block; // block tuple | 0 - block point | 1 - block frame
	Vec_double blockPoint;	  // point to intercept the puck and block it
	int blockFrame;			  // frame in the future at which the puck is to be intercepted

	block = findBlockPoint(trajectory, yblock);
	blockPoint = std::get<0>(block);
	blockFrame = std::get<1>(block);

	// compute hitting details
	double xlim = 10.0; 	  // minimum distance from right and left walls when computing swing path
	double ylim = 10.0;		  // minimum distance from the bottom wall when computing swing path
	double slope; 			  // slope of the line connecting puck hit point and puck target point
	Vec_double hitStartPoint; // start point of the hit path which is behind the puck aimed at the target
	Vec_double hitEndPoint;   // end point of the hit path which is past the puck on the line towards the target

	slope = (targetPoint.y - blockPoint.y)  / (targetPoint.x - blockPoint.x);

	if (slope < 0) // negative slope 
	{
		hitStartPoint.x = 66.0 - xlim;
		hitStartPoint.y = slope * (66.0 - xlim - blockPoint.x) + blockPoint.y;

		if (hitStartPoint.y < ylim) // check if y value is valid
		{
			hitStartPoint.y = ylim;
			hitStartPoint.x = ( ylim - blockPoint.y / slope ) + blockPoint.x;
		}
	}
	else  // positive slope
	{
		hitStartPoint.x = xlim;
		hitStartPoint.y = slope * (xlim - blockPoint.x) + blockPoint.y;

		if (hitStartPoint.y < ylim) // check if y value is valid
		{
			hitStartPoint.y = ylim;
			hitStartPoint.x = ( (ylim - blockPoint.y) / slope ) + blockPoint.x;
		}
	}

	hitEndPoint.x = blockPoint.x - (blockPoint.x - targetPoint.x) * 0.15; 
	hitEndPoint.y = slope * (hitEndPoint.x - blockPoint.x) + blockPoint.y;	

	// calculate number of steps
	double arrivalTime = blockFrame * sampleTime * 1000.0; 	// est. time in milliseconds
	int steps = int(arrivalTime / 2.0);						// est. number of steps (we assume each step takes ~2ms)

	// generate paths
	std::vector<Vec_double> initPath; // path from the current arm position to the strat of the hit path
	std::vector<Vec_double> hitPath;  // path from start point behind puck to point past the puck towards target
	std::vector<Vec_double> fullPath; // concatenation of the above two paths 

	initPath = computeLinearPath(arm2Pos, hitStartPoint, stepFactor*steps);
	hitPath  = computeLinearPath(hitStartPoint, hitEndPoint, (1.0-stepFactor)*steps);

	// this will catch if any of the composite paths are empty
	if (initPath.size() == 0)
	{
		return initPath;
	}
	else if (hitPath.size() == 0) 
	{
		return hitPath;
	}

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
	Puck puck(radius, 1.0, widthCm, heightCm);

	puck.updatePuck(points, 0.07);

	//std::cout << "vel cm/frame:  " << puck.getVelocity().x << " " << puck.getVelocity().y<< std::endl;
	//std::cout << "vel cm/second: " << puck.getVelocity().x * puck.getSampleTime() << " " << puck.getVelocity().y * puck.getSampleTime() << std::endl;

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

	//hitPath = Tiva.computeSwingPath(puck.getTrajectory(), targetPoint, puck.getSampleTime(), 20.0, 0.20);

	Vec_double startPoint, stopPoint;
	startPoint.x = 20;
	startPoint.y = 5;

	stopPoint.x = 5;
	stopPoint.y = 20;

	hitPath = Tiva.computeCurvedPath(startPoint, stopPoint, 100, 4.0);

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

	std::cout << "steps: " << hitPath.size() << std::endl;

	return 0;
}
*/

