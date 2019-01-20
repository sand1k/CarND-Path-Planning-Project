#include "path_planner.h"
#include "vehicle.h"
#include "spline.h"
#include <iostream>
using namespace std;

static Vehicle::State state = Vehicle::State::KL;
static double vel = 0.0; // reference velocity to target (m/s)
static double acc = 0.0; // reference accel
static int iter = 0;
static int tgt_lane = 1;

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

  cout << "--------------------- iter " << iter++ << " -------------------------" << endl;
  cout << "Cuurent: s=" << car_s << " d=" << car_d << " v= " << car_speed << " yaw=" << car_yaw << endl;
  cout << "Prev size: " << prev_size << endl;

  if (prev_size > 0)
  {
    car_s = end_path_s;
    car_d = end_path_d;
  }

  // Model
  Vehicle ego_vehicle(car_s, car_d, vel, acc, state, 0);
  cout << "Ego: s=" << car_s << " d=" << car_d << " v= " << vel << " acc=" << acc << endl;

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

    Vehicle vehicle(s, d, sqrt(vx * vx + vy * vy), 0, Vehicle::State::KL, v_id);
    printf("  veh %d l=%d s=%f d=%f v=%f\n", v_id, l, s, d, sqrt(vx * vx + vy * vy));
    predictions[v_id] = vehicle.generate_predictions(prev_size * 0.02);
    printf("  ->veh %d l=%d s=%f d=%f v=%f\n", v_id, l,
           predictions[v_id][0].get_s(), predictions[v_id][0].get_d(), predictions[v_id][0].get_v());
  }

  vector<Vehicle> trajectory = ego_vehicle.choose_next_state(tgt_lane, predictions);


  // Generate trajectory

  /*bool too_close = false;
  for (int i = 0; i < sensor_fusion.size(); i++)
  {
    float d = sensor_fusion[i][6];
    if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2))
    {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s += ((double) prev_size * .02 * check_speed);
      if ((check_car_s > car_s) && ((check_car_s - car_s) < 15))
      {
        //too_close = true;
        //if (lane > 0)
        //{
        //  lane = 0;
        //}
        //else {
        //  lane = 1;
        //}
        ref_vel = check_speed * 2.24;
      }
    }
  }

  if (too_close)
  {
    ref_vel -= .224;
  }
  else if (ref_vel < 49.5)
  {
    ref_vel += .224;
  }*/

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
    cout << "Ref yaw = " << ref_yaw << endl;

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  /*double delta = 30;
  vector<double> next_wp0 = getXY(car_s + delta, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s + 2 * delta, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s + 3 * delta, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);*/
  double new_s = trajectory[1].get_s();
  double new_d = trajectory[1].get_d();
  double new_v = trajectory[1].get_v();
  double new_a = trajectory[1].get_a();
  state = trajectory[1].get_state();
  vector<double> next_wp = getXY(new_s, new_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  ptsx.push_back(next_wp[0]);
  ptsy.push_back(next_wp[1]);

  for (int i = 0; i < ptsx.size(); i++)
  {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    double new_ptx = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    double new_pty = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

    printf("(%f, %f) -> (%f, %f)\n", ptsx[i], ptsy[i], new_ptx, new_pty);
    ptsx[i] = new_ptx;
    ptsy[i] = new_pty;
  }

  tk::spline s;
  s.set_points(ptsx, ptsy);

  for (int i = 0; i < prev_size; i++)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double target_x = 30;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);
  double x_add_on = 0;
  double current_speed = vel;
  for (int i = 0; i < 50 - prev_size; i++)
  {
    double x_point = x_add_on + current_speed * .02;
    if (x_point > target_x)
    {
      cout << "BAD PLANNING!" << endl;
      break;
    }
    double y_point = s(x_point);
    current_speed = current_speed + new_a * .02;

    /*double N = target_dist / (.02 * ref_vel / 2.24);
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);*/

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

    //printf("  pt (%f, %f)\n", x_point, y_point);
  }

  vel = current_speed;
  acc = new_a;
  tgt_lane = trajectory[1].get_lane();
}
