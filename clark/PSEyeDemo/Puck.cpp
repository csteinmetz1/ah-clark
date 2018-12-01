#include "stdafx.h"
#include "Puck.h"
#include <iostream>
#include <tuple>

// Constructor
Puck::Puck(double radius, double unitsPerCm, double widthCm, double heightCm)
{
	Vec_double initAcl;
	initAcl.x = 0.0;
	initAcl.y = 0.0;

	setRadius(radius);
	setUnitsPerCm(unitsPerCm);
	setRinkWidthCm(widthCm);
	setRinkHeightCm(heightCm);
}

// getter methods
double Puck::getSampleTime() const {return sampleTime;}
Vec_double Puck::getPosition() const {return pos;}
Vec_double Puck::getVelocity() const {return vel;}
Vec_double Puck::getAcceleration() const {return acl;}
std::vector<Vec_double> Puck::getTrajectory() { std::lock_guard<std::mutex> lock(mtx); return traj;}

// setter methods
void Puck::setSampleTime(double newSampleTime) {sampleTime = newSampleTime;}
void Puck::setPosition(Vec_double newPos) {pos = newPos;}
void Puck::setVelocity(Vec_double newVel) 
{
	vel = newVel;
	computeTrajectory();
}
void Puck::setAcceleration(Vec_double newAcl) {acl = newAcl;}
void Puck::setRadius(double newRadius) {radius = newRadius;}
void Puck::setUnitsPerCm(double newUnitsPerCm) {unitsPerCm = newUnitsPerCm;}
void Puck::setRinkWidthCm(double newRinkWidthCm) {rinkWidth = newRinkWidthCm * unitsPerCm;}
void Puck::setRinkHeightCm(double newRinkHeightCm) {rinkHeight = newRinkHeightCm * unitsPerCm;}

void Puck::updatePuck(std::vector<Vec_double> points, double sampleTime)
{
	setSampleTime(sampleTime);	  // set the avg time elapsed for a sample point
	setPosition(points.back());   // set current position to the last sampled value
	computeVelocity(points);	  // compute the updated velocity
	computeTrajectory();		  // update the trajectory based on new data
}

void Puck::move() 
{
	// update puck position
	pos.x += vel.x;
	pos.y += vel.y;

	// check puck position for wall collision *
	checkBoundary();
}

void Puck::checkBoundary()
{
	// right wall
	if (pos.x - radius < 0.0)
	{
		pos.x = 0.0 + radius;
		vel.x = -1.0 * vel.x;
	}
	// left wall
	else if (pos.x + radius > rinkWidth)
	{
		pos.x = rinkWidth - radius;
		vel.x = -1.0 * vel.x;
	}
	// bottom wall
	//else if (pos.y + radius < 0.0) {
	//	pos.y = 0.0 + radius;
	//	vel.y = -1.0 * vel.y;
	//}
	// top wall
	//else if (pos.y + radius > rinkHeight)
	//{
	//	pos.y = rinkHeight - radius;
	//	vel.y = -1.0 * vel.y;
	//}
	// our goal
	//else if (pos.y + radius > rinkHeight & pos.x < )
	//{
    //	pos.y = rinkHeight - radius;
	//	vel.y = -vel.y;
	//}
	// opponent goal
	//else if (pos.y + radius > rinkHeight)
	//{
	//	pos.y = rinkHeight - radius;
    //	vel.y = -vel.y;
    //}
}

void Puck::computeVelocity(std::vector<Vec_double> points) 
{
	std::lock_guard<std::mutex> lock(mtx);
	auto fit = leastSquaresFit(points);

	Vec_double initPos, finalPos;

	initPos.y = points.front().y; 
	initPos.x = ( initPos.y - std::get<0>(fit) ) / std::get<1>(fit);

	//std::cout << initPos.x << " " << initPos.y << std::endl;

	finalPos.y = points.back().y; 
	finalPos.x = ( finalPos.y - std::get<0>(fit) ) / std::get<1>(fit);

	//std::cout << finalPos.x << " " << finalPos.y << std::endl;

	vel.x = (finalPos.x - initPos.x) / double(points.size()-1);
	vel.y = (finalPos.y - initPos.y) / double(points.size()-1);
}

void Puck::computeTrajectory() 
{
	//std::vector<Vec_double> trajectory;
	//for ( int frame = 0; frame < estimation_size; frame++ ) {
	//	move();
	//	trajectory.push_back(getPosition());
	//}

	std::lock_guard<std::mutex> lock(mtx);
	traj.clear(); // clear trajectory predicitions
	Vec_double current_pos = pos; // hold future position
	Vec_double current_vel = vel; // hold future velocity

	while ( current_pos.y > 0 & traj.size() < 100 )
	{
		current_pos.x += current_vel.x;
		current_pos.y += current_vel.y;

		// right wall
		if (current_pos.x - radius < 0.0)
		{
			current_pos.x = 0.0 + radius;
			current_vel.x = -0.75 * current_vel.x;
		}
		// left wall
		else if (current_pos.x + radius > rinkWidth)
		{
			current_pos.x = rinkWidth - radius;
			current_vel.x = -0.75 * current_vel.x;
		}

		traj.push_back(current_pos);
	}
}

std::tuple<double, double> Puck::leastSquaresFit(std::vector<Vec_double> points)
{
	std::vector<double> xy;
	std::vector<double> x_sqrd;
	std::vector<double> y_sqrd;

	double n = points.size();
	double sum_x = 0;
	double sum_y = 0;
	double sum_xy = 0;
	double sum_x_sqrd = 0;
	double sum_y_sqrd = 0;

	for (auto point : points) 
	{
		sum_x += point.x;
		sum_y += point.y;
		xy.push_back( point.x * point.y );
		x_sqrd.push_back( pow(point.x, 2) );
		y_sqrd.push_back( pow(point.y, 2) );
	}

	for (auto val : xy) 
	{
		sum_xy += val;
	}

	for (auto val : x_sqrd)
	{
		sum_x_sqrd += val;
	}

	for (auto val : y_sqrd)
	{
		sum_y_sqrd += val;
	}

	// find model parameters
	double a;
	double b;

	a = ( (sum_y * sum_x_sqrd) - (sum_x * sum_xy) ) / ( (n * sum_x_sqrd) - pow(sum_x, 2) );
	b = ( (n * sum_xy) - (sum_x * sum_y) ) / ( (n * sum_x_sqrd) - pow(sum_x, 2) );

	auto fit = std::make_tuple(a, b);

	return fit;
}

