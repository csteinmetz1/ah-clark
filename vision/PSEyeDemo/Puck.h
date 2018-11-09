#ifndef PUCK_H
#define PUCK_H

#include <vector>
#include <cmath>

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
		void computeVelocity(std::vector<Vec_double> points);
		std::vector<Vec_double> computeTrajectory(int estimation_size);
		static std::tuple<double, double> leastSquaresFit(std::vector<Vec_double> points);

		// Constructor
		Puck(std::vector<Vec_double> points, Vec_double initAcl, double radius, 
		   double unitsPerCm, double widthCm, double heightCm);

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
