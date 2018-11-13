#ifndef PUCK_H
#define PUCK_H

#include <vector>
#include <cmath>
#include <mutex>

struct Vec_double {
	double x;
	double y;
};

class Puck
{
	public:
		// getter methods
		double getSampleTime() const;
		Vec_double getPosition() const;
		Vec_double getVelocity() const;
		Vec_double getAcceleration() const;
		std::vector<Vec_double> getTrajectory();

		// setter methods
		void setSampleTime(double newSampleTime);
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
		void updatePuck(std::vector<Vec_double> points, double sampleTime);
		void computeVelocity(std::vector<Vec_double> points);
		void computeTrajectory();
		static std::tuple<double, double> leastSquaresFit(std::vector<Vec_double> points);

		// Constructor
		Puck(double radius, double unitsPerCm, double widthCm, double heightCm);

	private:
		Vec_double pos;
		Vec_double vel;
		Vec_double acl;
		std::vector<Vec_double> traj;
		double sampleTime;
		double unitsPerCm;
		double radius;
		double rinkWidth;
		double rinkHeight;
		std::mutex mtx;
};

#endif
