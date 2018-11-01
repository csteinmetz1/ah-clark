#ifndef DRAW_H
#define DRAW_H

#include <tuple>
#include <string>
#include <iostream> 
#include <cmath>
#include "Puck.h"

class Draw 
{
	public:
		static std::vector<Vec_double> Circle(double radius, int steps, Vec_double offset);
		static std::vector<Vec_double> Square(double side, Vec_double offset);
		static std::vector<Vec_double> Weird(double start, double stop, int steps, Vec_double offset);
};

#endif