#include "path_planner.h"
#include "vehicle.h"
#include "spline.h"
#include <iostream>
using namespace std;

static Vehicle::State g_state = Vehicle::State::KL; // current state
static double g_velocity = 0.0;                     // reference velocity to target (m/s)
static double g_acceleration = 0.0;                 // reference accel
static int g_target_lane = 1;                       // target lane

void plan_path(double car_x, double car_y,
               double car_s, double car_d,
               double car_yaw, double car_speed,
               const vector<double> previous_path_x,
               const vector<double> previous_path_y,
               double end_path_s,
               double end_path_d,
               const vector<vector<double>> &sensor_fusion,
               vector<double> &next_x_vals, vector<double> &next_y_vals)
{
  int prev_size = previous_path_x.size();
  if (prev_size > 0)
  {
    car_s = end_path_s;
    car_d = end_path_d;
  }

  //
  // Choose trajectory
  //
  Vehicle ego_vehicle(g_target_lane, car_s, car_d, g_velocity, g_acceleration, g_state, 0);

  Vehicle::Predictions predictions;
  for (vector<vector<double>>::const_iterator it = sensor_fusion.begin(); it != sensor_fusion.cend(); ++it)
  {
    int v_id = static_cast<int>((*it)[0]);
    double x = (*it)[1];
    double y = (*it)[2];
    double vx = (*it)[3];
    double vy = (*it)[4];
    double s = (*it)[5];
    double d = (*it)[6];
    int l = d / 4;

    Vehicle vehicle(l, s, d, sqrt(vx * vx + vy * vy), 0, Vehicle::State::KL, v_id);
    predictions[v_id] = vehicle.generate_predictions(prev_size * 0.02);
  }

  vector<Vehicle> trajectory = ego_vehicle.choose_next_state(predictions);

  //
  // Generate spline according to selected trajectory
  //
  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  if (prev_size < 2)
  {
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }
  else
  {
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // Get new state of the vehicle
  double new_s = trajectory[0].get_s();
  double new_d = trajectory[0].get_d();
  double new_v = trajectory[0].get_v();
  g_target_lane = trajectory[0].get_lane();
  g_acceleration = trajectory[0].get_a();
  g_state = trajectory[0].get_state();

  // Add waypoint
  vector<double> next_wp1 = getXY(new_s, new_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  ptsx.push_back(next_wp1[0]);
  ptsy.push_back(next_wp1[1]);

  // Add artificial waypoint to smooth the trjaectory
  next_wp1 = getXY(new_s + 2 * (new_s - car_s), new_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  ptsx.push_back(next_wp1[0]);
  ptsy.push_back(next_wp1[1]);

  // Convert to local coordinate system
  for (int i = 0; i < ptsx.size(); i++)
  {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    double new_ptx = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    double new_pty = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

    ptsx[i] = new_ptx;
    ptsy[i] = new_pty;
  }

  tk::spline s;
  s.set_points(ptsx, ptsy);

  // Push back previous points
  for (int i = 0; i < prev_size; i++)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // Generate new points
  double x_add_on = 0;
  double current_speed = g_velocity;
  for (int i = 0; i < 50 - prev_size; i++)
  {
    double x_point = x_add_on + current_speed * .02;
    double y_point = s(x_point);
    current_speed = current_speed + g_acceleration * .02;

    x_add_on = x_point;

    // convert to global coordinates
    double x_ref = x_point;
    double y_ref = y_point;
    x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));
    x_point += ref_x;
    y_point += ref_y;

    // push point to trajectory
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  // update global velocity according to last added point
  g_velocity = current_speed;
}
