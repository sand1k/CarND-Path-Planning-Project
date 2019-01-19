#include "vehicle.h"

Vehicle::Vehicle(Vehicle::State state, int lane,
                 double x, double y, double s, double d, double yaw, double speed)
{
  state_ = state;
  lane_ = lane;
  x_ = x;
  y_ = y;
  s_ = s;
  d_ = d;
  yaw_ = yaw;
  speed_ = speed;
}

vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {

    vector<string> states = successor_states();
    float cost;
    vector<float> costs;
    vector<vector<Vehicle>> final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
        if (trajectory.size() != 0) {
            cost = calculate_cost(*this, predictions, trajectory);
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
        }
    }

    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    return final_trajectories[best_idx];
}

vector<Vehicle::State> Vehicle::successor_states()
{
  vector<State> states;
  states.push_back(State::KL);
  switch (lane_)
  {
  case 0:
    states.push_back(State::PLCR);
    states.push_back(State::LCR);
    break;
  case 1:
    states.push_back(State::PLCL);
    states.push_back(State::PLCR);
    states.push_back(State::LCL);
    states.push_back(State::LCR);
    break;
  case 2:
    states.push_back(State::PLCL);
    states.push_back(State::LCL);
    break;
  }
  return states;
}
