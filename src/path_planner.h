#pragma once

#include <vector>
#include "math.h"
using namespace std;

constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

void plan_path(double car_x, double car_y,
               double car_s, double car_d,
               double car_yaw, double car_speed,
               vector<double> &next_x_vals, vector<double> &next_y_vals);
