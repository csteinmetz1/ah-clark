#include "stdafx.h"
#include "Tiva.h"
#include "Draw.h"

std::vector<Vec_double> Draw::Circle(double radius, int steps, Vec_double offset) {

	double u;
	Vec_double point;
	std::vector<Vec_double> path;
	double pi = 3.14159265358979323846;

	u = 0;

	while (u < pi) {
		point.x = radius * cos(u) + offset.x;
		point.y = radius * sin(u) + offset.y;
		path.push_back(point);
		u += (pi / steps);
	}
	return path;
}

std::vector<Vec_double> Draw::Weird(double start, double stop, int steps, Vec_double offset) {

	double u;
	Vec_double point;
	std::vector<Vec_double> path;

	u = start;

	while (u < stop) {
		point.x = (u + 2 * sin(2 * u)) + offset.x;
		point.y = 6 * sin(u) + offset.y;
		path.push_back(point);
		u += (stop / steps);
	}
	return path;
}