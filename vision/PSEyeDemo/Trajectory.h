#include <tuple>
#include <string>
#include <vector>
#include <iostream>
#include "Puck.h"

Vec computeVelocity(Vec init_pos, Vec final_pos, int frames);
Vec computeAcceleration(Vec init_vel, Vec final_vel, int frames);
std::vector<Vec> computeTrajectory(Vec p, Vec v, Vec a, int estimation_size);