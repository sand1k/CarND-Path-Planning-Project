#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <map>
#include <algorithm>
using namespace std;

class Vehicle
{
public:
  enum State
  {
      PLCL,
      PLCR,
      LCL,
      LCR,
      KL
  };

  typedef map<int, vector<Vehicle>> Predictions;
  typedef vector<Vehicle> Trajectory;

public:

  Vehicle() {}
  Vehicle(State state, int lane, double x, double y, double s, double d, double speed);

  Trajectory choose_next_state(const Predictions &predictions);
  Trajectory generate_predictions(int horizon = 2);
private:
  vector<State> successor_states();
  Trajectory generate_trajectory(State state, const Predictions &predictions);

private:
  int lane_;
  State state_;
  double x_;
  double y_;
  double s_;
  double d_;
  double speed_;
  double accel_;
};

float calculate_cost(const Vehicle &vehicle, const Vehicle::Predictions &predictions,
                     const Vehicle::Trajectory &trajectory);

#endif
