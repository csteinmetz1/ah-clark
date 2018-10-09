#include <tuple>
#include <string>

using namespace std;

struct Point {
	int x;
	int y;
};

tuple<double,double> computeVelocity(Point init_pos, Point final_pos, int frames);
tuple<double,double> computeAcceleration(double init_vel, double final_vel, int frames);
