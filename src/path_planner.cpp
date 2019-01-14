#include "path_planner.h"

void plan_path(double car_x, double car_y,
               double car_s, double car_d,
               double car_yaw, double car_speed,
               vector<double> &next_x_vals, vector<double> &next_y_vals)
{
  double dist_inc = 0.5;
  for(int i = 0; i < 50; i++)
  {
    next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
    next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
  }
}
