#include <vector>

struct Comp {
	double x;
	double y;
};

class Puck
{
	public:
		// getter methods
		Comp getPosition() const;
		Comp getVelocity() const;
		Comp getAcceleration() const;

		// setter methods
		void setPosition(Comp newPos);
		void setVelocity(Comp newVel);
		void setAcceleration(Comp newAcl);
		void setRadius(double newRadius);
		void setUnitsPerCm(double newUnitsPerCm);
		void setRinkWidthCm(double newRinkWidthCm);
		void setRinkHeightCm(double newRinkHeightCm);

		// useful methods
		void move();
		void checkBoundary();
		void computeVelocity(Comp init_pos, Comp final_pos, int frames);
		std::vector<Comp> computeTrajectory(int estimation_size);

		// Constructor
		Puck(Comp firstPos, Comp secondPos, Comp initAcl, double radius, 
		   double unitsPerCm, double widthCm, double heightCm, int frames);

	private:
		Comp pos;
		Comp vel;
		Comp acl;
		double unitsPerCm;
		double radius;
		double rinkWidth;
		double rinkHeight;
};