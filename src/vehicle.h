#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <map>
#include <algorithm>
#include <float.h>
#include <iostream>
using namespace std;

#define PREFERRED_BUFFER 15.0f       // Preferred buffer to the next car in meters
#define TARGET_SPEED 22.1f           // ~ 49.5 mph
#define PASS_GAP 15.0f               // Safe distance to the front and rear vehicle in the adjacent lane
                                     // to start lane change
#define MAX_ACCEL 9.0f               // Max acceleration
#define MAX_N_ACCEL 2.5f             // Max acceleration across the road
#define MID_LANE_DIST_PRIORITY 20.0f // I consider that middle lane is preferrable because it has more room
                                     // for maneuvers. Unless distance to the next vehicle in the left or right
                                     // lanes is greater than distance to the next vehicle in the middle lane
                                     // plus MID_LANE_DIST_PRIORITY, I choose the middle lane.
#define OBSERVABLE_DISTANCE 100.0    // How far the vehicle can see

//
// Class to predict/choose vehicle trajectory
//
class Vehicle
{
public:
  // Possible states
  enum State
  {
      LCL, // Lane change left
      LCR, // Lane change right
      KL,  // Keep lane
      STATES_COUNT
  };

  typedef vector<Vehicle> Trajectory; // A set of next positions of vehicle (in this project I use only one next
                                      // position)
  typedef map<int, Trajectory> Predictions; // Trajectories of different vehicles

public:

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
  bool get_vehicle_ahead(const Predictions &predictions, int lane, const Vehicle **rVehicle) const;

private:
  vector<State> successor_states();
  Trajectory generate_trajectory(State state, const Predictions &predictions);
  Trajectory constant_speed_trajectory();
  Trajectory keep_lane_trajectory(const Predictions &predictions);
  Trajectory lane_change_trajectory(State state, const Predictions &predictions);
  vector<float> get_kinematics(const Predictions &predictions, int lane);

private:
  int tgt_lane_; // target lane - a lane that we should follow
  int lane_;     // current lane (calculated according to value of d_
  State state_;  // current state
  double s_;     // longitudal position
  double d_;     // lateral positon
  double v_;     // speed
  double a_;     // accelration
  int id_;       // vehicle id

  static int lane_direction_[STATES_COUNT];      // increment to the current lane according 
  static const char *state_names_[STATES_COUNT]; // names of states
};

float calculate_cost(const Vehicle &vehicle, const Vehicle::Predictions &predictions,
                     const Vehicle::Trajectory &trajectory);

#endif
