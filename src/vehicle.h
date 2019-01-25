#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <map>
#include <algorithm>
#include <float.h>
#include <iostream>
using namespace std;

#define PREFERRED_BUFFER 15.0f   // preferred buffer to the next car in meters
#define TARGET_SPEED 22.1f      // ~ 49.5 mph
#define PASS_GAP 15.0f
#define MAX_ACCEL 9.0f
#define MAX_N_ACCEL 2.5f

class Vehicle
{
public:
  enum State
  {
      LCL,
      LCR,
      KL
  };

  typedef map<int, vector<Vehicle>> Predictions;
  typedef vector<Vehicle> Trajectory;

public:

  //Vehicle() {}
  Vehicle(int tgt_lane, double s, double d, double speed, double accel, State state, int id);
  double get_s() const { return s_; }
  double get_d() const  { return d_; }
  double get_v() const { return v_; }
  double get_a() const { return a_; }
  int get_id() const { return id_; }
  State get_state() const { return state_; }
  int get_lane() const { return lane_; }
  int get_tgt_lane() const { return tgt_lane_; }

  Trajectory choose_next_state(const Predictions &predictions);
  Trajectory generate_predictions(float dt);
  bool get_vehicle_behind(const Predictions &predictions, int lane, const Vehicle **rVehicle) const;
  bool get_vehicle_ahead(const Predictions &predictions, int lane, const Vehicle **rVehicle) const;
private:
  vector<State> successor_states();
  Trajectory generate_trajectory(State state, const Predictions &predictions);
  float position_at(float t);
  Trajectory constant_speed_trajectory();
  Trajectory keep_lane_trajectory(const Predictions &predictions);
  Trajectory lane_change_trajectory(State state, const Predictions &predictions);
  vector<float> get_kinematics(const Predictions &predictions, int lane);

private:
  int tgt_lane_;
  int lane_;
  State state_;
  double s_;
  double d_;
  double v_;
  double a_;
  int id_;

public:
  static int lane_direction_[4];
  static const char *state_names_[6];
};

float calculate_cost(const Vehicle &vehicle, const Vehicle::Predictions &predictions,
                     const Vehicle::Trajectory &trajectory);

#endif
