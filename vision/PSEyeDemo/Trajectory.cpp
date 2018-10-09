#include "Trajectory.h"

tuple<double,double> computeVelocity(Point init_pos, Point final_pos, int frames) {

	int	x_vel = (final_pos.x - init_pos.x) / frames;
	int y_vel = (final_pos.y - init_pos.y) / frames;

	return make_tuple(x_vel, y_vel)
}