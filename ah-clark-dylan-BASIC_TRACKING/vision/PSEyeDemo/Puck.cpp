#include "stdafx.h"
#include "Puck.h"
#include <iostream>

// Constructor
Puck::Puck(Vec_double initPos, Vec_double secondPos, Vec_double initAcl, double radius,
	double unitsPerCm, double widthCm, double heightCm, int frames) {
	setPosition(secondPos);
	computeVelocity(initPos, secondPos, frames);
	setAcceleration(initAcl);
	setRadius(radius);
	setUnitsPerCm(unitsPerCm);
	setRinkWidthCm(widthCm);
	setRinkHeightCm(heightCm);
}

// getter methods
Vec_double Puck::getPosition() const { return pos; }
Vec_double Puck::getVelocity() const { return vel; }
Vec_double Puck::getAcceleration() const { return acl; }

// setter methods
void Puck::setPosition(Vec_double newPos) { pos = newPos; }
void Puck::setVelocity(Vec_double newVel) { vel = newVel; }
void Puck::setAcceleration(Vec_double newAcl) { acl = newAcl; }
void Puck::setRadius(double newRadius) { radius = newRadius; }
void Puck::setUnitsPerCm(double newUnitsPerCm) { unitsPerCm = newUnitsPerCm; }
void Puck::setRinkWidthCm(double newRinkWidthCm) { rinkWidth = newRinkWidthCm * unitsPerCm; }
void Puck::setRinkHeightCm(double newRinkHeightCm) { rinkHeight = newRinkHeightCm * unitsPerCm; }

void Puck::move()
{
	// update puck position
	pos.x += vel.x;
	pos.y += vel.y;

	// update puck velocity
	vel.x += acl.x;
	vel.y += acl.y;

	// check puck position for wall collision
	checkBoundary();

}

void Puck::checkBoundary()
{
	// right wall
	if (pos.x + radius < 0.0)
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
	else if (pos.y + radius < 0.0) {
		pos.y = 0.0 + radius;
		vel.y = -1.0 * vel.y;
	}
	// top wall
	else if (pos.y + radius > rinkHeight)
	{
		pos.y = rinkHeight - radius;
		vel.y = -1.0 * vel.y;
	}
	// our goal
	//else if (pos.y + radius > rinkHeight & pos.x < )
	//{
	//		pos.y = rinkHeight - radius;
	//	vel.y = -vel.y;
	//}
	// opponent goal
	//else if (pos.y + radius > rinkHeight)
	//{
	//	pos.y = rinkHeight - radius;
	//	vel.y = -vel.y;
	//}
}

void Puck::computeVelocity(Vec_double init_pos, Vec_double final_pos, int frames) {

	vel.x = (final_pos.x - init_pos.x) / frames;
	vel.y = (final_pos.y - init_pos.y) / frames;

}

std::vector<Vec_double> Puck::computeTrajectory(int estimation_size) {

	std::vector<Vec_double> trajectory;

	for (int frame = 0; frame < estimation_size; frame++) {
		// update current position based on velocity

		move();
		trajectory.push_back(getPosition());

		//current_pos.x = current_pos.x + current_vel.x;
		//current_pos.y = current_pos.y + current_vel.y;
		//trajectory.push_back(current_pos);

		// update current velocity based on acceleration
		//current_vel.x = current_vel.x + accel.x;
		//current_vel.y = current_vel.x + accel.y;
	}

	return trajectory;
}