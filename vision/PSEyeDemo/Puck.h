#include <vector>

struct Vec {
	double x;
	double y;
};

class Puck
{
	public:
		// getter methods
		Vec getPosition() const;
		Vec getVelocity() const;
		Vec getAcceleration() const;

		// setter methods
		void setPosition(Vec newPos);
		void setVelocity(Vec newVel);
		void setAcceleration(Vec newAcl);
		void setRadius(double newRadius);
		void setUnitsPerCm(double newUnitsPerCm);
		void setRinkWidthCm(double newRinkWidthCm);
		void setRinkHeightCm(double newRinkHeightCm);

		// useful methods
		void move();
		void checkBoundary();
		void computeVelocity(Vec init_pos, Vec final_pos, int frames);
		std::vector<Vec> computeTrajectory(int estimation_size);

		// Constructor
		Puck(Vec firstPos, Vec secondPos, Vec initAcl, double radius, 
		   double unitsPerCm, double widthCm, double heightCm, int frames);

	private:
		Vec pos;
		Vec vel;
		Vec acl;
		double unitsPerCm;
		double radius;
		double rinkWidth;
		double rinkHeight;
};