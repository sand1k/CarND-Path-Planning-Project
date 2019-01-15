#pragma once

#include <vector>
#include "math.h"
using namespace std;

constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

// Load up map values for waypoint's x,y,s and d normalized normal vectors
extern vector<double> map_waypoints_x;
extern vector<double> map_waypoints_y;
extern vector<double> map_waypoints_s;
extern vector<double> map_waypoints_dx;
extern vector<double> map_waypoints_dy;

vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

void plan_path(double car_x, double car_y,
               double car_s, double car_d,
               double car_yaw, double car_speed,
               const vector<double> previous_path_x,
               const vector<double> previous_path_y,
               double end_path_s,
               double end_path_d,
               const vector<vector<double>> &sensor_fusion,
               vector<double> &next_x_vals, vector<double> &next_y_vals);
