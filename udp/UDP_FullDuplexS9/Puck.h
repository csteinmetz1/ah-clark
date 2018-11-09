#pragma once
#ifndef PUCK_H
#define PUCK_H

#include <vector>

struct Vec_double {
	double x;
	double y;
};

class Puck
{
public:
	// getter methods
	Vec_double getPosition() const;
	Vec_double getVelocity() const;
	Vec_double getAcceleration() const;

	// setter methods
	void setPosition(Vec_double newPos);
	void setVelocity(Vec_double newVel);
	void setAcceleration(Vec_double newAcl);
	void setRadius(double newRadius);
	void setUnitsPerCm(double newUnitsPerCm);
	void setRinkWidthCm(double newRinkWidthCm);
	void setRinkHeightCm(double newRinkHeightCm);

	// useful methods
	void move();
	void checkBoundary();
	void computeVelocity(Vec_double init_pos, Vec_double final_pos, int frames);
	std::vector<Vec_double> computeTrajectory(int estimation_size);

	// Constructor
	Puck(Vec_double initPos, Vec_double secondPos, Vec_double initAcl, double radius,
		double unitsPerCm, double widthCm, double heightCm, int frames);

private:
	Vec_double pos;
	Vec_double vel;
	Vec_double acl;
	double unitsPerCm;
	double radius;
	double rinkWidth;
	double rinkHeight;
};

#endif