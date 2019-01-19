#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
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

  Vehicle(State state, int lane, double x, double y, double s, double d, double yaw, double speed);

  vector<State> successor_states();

private:
  int lane_;
  State state_;
  double x_;
  double y_;
  double s_;
  double d_;
  double yaw_;
  double speed_;
};

#endif
