#include "Puck.h"
#include <iostream>

// Constructor
Puck::Puck(Comp firstPos, Comp secondPos, Comp initAcl, double radius, 
		   double unitsPerCm, double widthCm, double heightCm, int frames) {
	setPosition(secondPos);
	computeVelocity(firstPos, secondPos, frames);
	setAcceleration(initAcl);
	setRadius(radius);
	setUnitsPerCm(unitsPerCm);
	setRinkWidthCm(widthCm);
	setRinkHeightCm(heightCm);
}

// getter methods
Comp Puck::getPosition() const {return pos;}
Comp Puck::getVelocity() const {return vel;}
Comp Puck::getAcceleration() const {return acl;}

// setter methods
void Puck::setPosition(Comp newPos) {pos = newPos;}
void Puck::setVelocity(Comp newVel) {vel = newVel;}
void Puck::setAcceleration(Comp newAcl) {acl = newAcl;}
void Puck::setRadius(double newRadius) {radius = newRadius;}
void Puck::setUnitsPerCm(double newUnitsPerCm) {unitsPerCm = newUnitsPerCm;}
void Puck::setRinkWidthCm(double newRinkWidthCm) {rinkWidth = newRinkWidthCm * unitsPerCm;}
void Puck::setRinkHeightCm(double newRinkHeightCm) {rinkHeight = newRinkHeightCm * unitsPerCm;}

void Puck::move() 
{
	// update puck position
	pos.x += vel.x;
	pos.y += vel.y;

	// update puck velocity
	vel.x += acl.x;
	vel.y += acl.y;

	// check puck position for wall collision *
	checkBoundary();
}

void Puck::checkBoundary()
{
	// right wall
	if 	    (pos.x + radius < 0.0) 
	{
		pos.x = 0.0 + radius;
		vel.x = -vel.x;
	} 
	// left wall
	else if (pos.x + radius > rinkWidth)
	{
		pos.x = rinkWidth - radius;
		vel.x = -vel.x;
	}
	// bottom wall
	else if (pos.y + radius < 0.0){
		pos.y = 0.0 + radius;
		vel.y = -vel.y;
	}
	// top wall
	else if (pos.y + radius > rinkHeight)
	{
		pos.y = rinkHeight - radius;
		vel.y = -vel.y;
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

void Puck::computeVelocity(Comp init_pos, Comp final_pos, int frames) {

	vel.x = (final_pos.x - init_pos.x) / frames;
	vel.y = (final_pos.y - init_pos.y) / frames;

}

std::vector<Comp> Puck::computeTrajectory(int estimation_size) {

	std::vector<Comp> trajectory;

	for ( int frame = 0; frame < estimation_size; frame++ ) {
		move();
		trajectory.push_back(getPosition());
	}

	return trajectory;
}
