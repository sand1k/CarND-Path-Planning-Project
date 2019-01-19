#include "vehicle.h"

Vehicle::Vehicle(Vehicle::State state, int lane,
                 double x, double y, double s, double d, double speed)
{
  state_ = state;
  lane_ = lane;
  x_ = x;
  y_ = y;
  s_ = s;
  d_ = d;
  speed_ = speed;
}

vector<Vehicle> Vehicle::choose_next_state(const map<int, vector<Vehicle>> &predictions)
{

  vector<State> states = successor_states();
  float cost;
  vector<float> costs;
  vector<vector<Vehicle>> final_trajectories;

  for (vector<State>::iterator it = states.begin(); it != states.end(); ++it) {
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

Vehicle::Trajectory Vehicle::generate_predictions(int horizon) {
  /*
     Generates predictions for non-ego vehicles to be used
     in trajectory generation for the ego vehicle.
   */
  vector<Vehicle> predictions;
  for (int i = 0; i < horizon; i++) {
    /*float next_s = position_at(i);
    float next_v = 0;
    if (i < horizon-1) {
      next_v = position_at(i+1) - s;
    }
    predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));*/
  }
  return predictions;
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


Vehicle::Trajectory Vehicle::generate_trajectory(Vehicle::State state, const Vehicle::Predictions &predictions)
{
  return vector<Vehicle>();
}

float calculate_cost(const Vehicle &vehicle, const Vehicle::Predictions &predictions,
                     const Vehicle::Trajectory &trajectory)
{
  return 1.0;
}
