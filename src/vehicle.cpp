#include "vehicle.h"
#include <assert.h>
#include <math.h>


int Vehicle::lane_direction_[Vehicle::STATES_COUNT] = {-1, 1, 0};
const char *Vehicle::state_names_[Vehicle::STATES_COUNT] = {"LCL", "LCR", "KL"};

//
// Helper function which returns lateral coordinate of lane center
//
static float lane_center(int lane)
{
  return 2.0 + 4.0 * lane;
}

//
// Vehicle constructor
//
Vehicle::Vehicle(int tgt_lane, double s, double d, double speed, double accel, Vehicle::State state, int id)
{
  state_ = state;
  lane_ = d / 4;
  tgt_lane_ = tgt_lane;
  s_ = s;
  d_ = d;
  v_ = speed;
  a_ = accel;
  id_ = id;
}

//
// Choose next state of the vehicle according to predicted positions of other vehicles
//
vector<Vehicle> Vehicle::choose_next_state(const Predictions &predictions)
{
  vector<State> states = successor_states();
  float cost;
  vector<float> costs;
  vector<Vehicle::Trajectory> final_trajectories;

  for (vector<State>::iterator it = states.begin(); it != states.end(); ++it)
  {
    vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
    if (trajectory.size() != 0)
    {
      cost = calculate_cost(*this, predictions, trajectory);
      costs.push_back(cost);
      final_trajectories.push_back(trajectory);
    }
  }

  vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);
  return final_trajectories[best_idx];
}

//
// Generates predictions for non-ego vehicles to be used
// in trajectory generation for the ego vehicle.
//
Vehicle::Trajectory Vehicle::generate_predictions(float dt)
{
  vector<Vehicle> predictions;

  float next_s = s_ + v_ * dt;
  float next_v = v_;

  float next_d = lane_center(lane_);
  predictions.push_back(Vehicle(tgt_lane_, next_s, next_d, next_v, 0.0, KL, id_));

  return predictions;
}

//
// Return state of possible successor states
//
vector<Vehicle::State> Vehicle::successor_states()
{
  vector<State> states;
  states.push_back(State::KL);

  switch (lane_)
  {
  case 0:
    states.push_back(State::LCR);
    break;
  case 1:
    states.push_back(State::LCL);
    states.push_back(State::LCR);
    break;
  case 2:
    states.push_back(State::LCL);
    break;
  }
  return states;
}


//
// Given a possible next state, generate the appropriate trajectory to realize the next state.
//
Vehicle::Trajectory Vehicle::generate_trajectory(Vehicle::State state, const Vehicle::Predictions &predictions)
{
  Trajectory trajectory;
  if (state == KL)
  {
    trajectory = keep_lane_trajectory(predictions);
  }
  else if (state == LCL || state == LCR)
  {
    trajectory = lane_change_trajectory(state, predictions);
  }

  return trajectory;
}

//
// Generate a keep lane trajectory.
//
Vehicle::Trajectory Vehicle::keep_lane_trajectory(const Predictions &predictions)
{
  Trajectory trajectory;
  vector<float> kinematics = get_kinematics(predictions, tgt_lane_);
  float new_s = kinematics[0];
  float new_d = kinematics[1];
  float new_v = kinematics[2];
  float new_a = kinematics[3];
  trajectory.push_back(Vehicle(tgt_lane_, new_s, new_d, new_v, new_a, KL, 0));
  return trajectory;
}

//
// Generate a lane change trajectory.
//
Vehicle::Trajectory Vehicle::lane_change_trajectory(State state, const Predictions &predictions)
{
  int new_lane = tgt_lane_ + lane_direction_[state];
  Trajectory trajectory;
  //Check if a lane change is possible (check if another vehicle occupies that spot).
  for (Predictions::const_iterator it = predictions.cbegin(); it != predictions.cend(); ++it)
  {
    const Vehicle &next_lane_vehicle = it->second[0];
    if (next_lane_vehicle.lane_ == new_lane
        && next_lane_vehicle.s_ > s_ - PASS_GAP * 0.75 && next_lane_vehicle.s_ < s_ + PASS_GAP)
    {
      //If lane change is not possible, return empty trajectory.
      return trajectory;
    }
  }
  vector<float> kinematics = get_kinematics(predictions, new_lane);
  trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], kinematics[3], state, 0));
  return trajectory;
}


//
// Gets next timestep kinematics (s, d, velocity)
// for a given lane. Tries to choose the maximum velocity and acceleration,
// given other vehicle positions and accel/velocity constraints.
//
vector<float> Vehicle::get_kinematics(const Vehicle::Predictions &predictions, int new_lane)
{
  float max_velocity_accel_limit = MAX_ACCEL + v_;
  float min_velocity_accel_limit = max(-MAX_ACCEL + v_, 0.0);
  float new_position;
  float new_velocity;
  float new_accel;
  const Vehicle *veh_ahe;
  const Vehicle *veh_beh;

  if (get_vehicle_ahead(predictions, lane_, &veh_ahe))
  {
    float max_velocity_in_front = (veh_ahe->s_ - s_ - PREFERRED_BUFFER) + veh_ahe->v_;
    new_velocity = min(max_velocity_in_front, max_velocity_accel_limit);
    new_velocity = min(new_velocity, TARGET_SPEED);
    new_velocity = max(min_velocity_accel_limit, new_velocity);
  }
  else
  {
    new_velocity = min(max_velocity_accel_limit, TARGET_SPEED);
  }

  float new_d = lane_center(new_lane);
  if (abs(new_d - d_) > MAX_N_ACCEL)
  {
    new_d = d_ + copysign(MAX_N_ACCEL, new_d - d_);
  }
  new_accel = new_velocity - v_; //Equation: (v_1 - v_0)/t = acceleration
  new_position = s_ + new_velocity + new_accel / 2.0;

  return {new_position, new_d, new_velocity, new_accel};
}

//
// Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
// rVehicle is updated if a vehicle is found.
///
bool Vehicle::get_vehicle_ahead(const Vehicle::Predictions &predictions, int lane, const Vehicle **rVehicle) const
{
  float min_s = FLT_MAX;
  bool found_vehicle = false;
  for (Predictions::const_iterator it = predictions.cbegin(); it != predictions.cend(); ++it)
  {
    const Vehicle &temp_vehicle = it->second[0];
    if (temp_vehicle.lane_ == lane && temp_vehicle.s_ > s_ && temp_vehicle.s_ < min_s)
    {
      min_s = temp_vehicle.s_;
      (*rVehicle) = &it->second[0];
      found_vehicle = true;
    }
  }
  return found_vehicle;
}

//
// Returns distance to the vehicle in case it is less than OBSERVABLE_DISTANCE and FLT_MAX otherwise
//
static float get_truncated_dist_to_vehicle(int lane, const Vehicle &vehicle, const Vehicle::Predictions &predictions)
{
  const Vehicle *veh_ahe;
  float dist = FLT_MAX;
  if (vehicle.get_vehicle_ahead(predictions, lane, &veh_ahe))
  {
    dist = veh_ahe->get_s() - vehicle.get_s();
  }

  return (dist < OBSERVABLE_DISTANCE) ? dist : 300;
}

//
// Calculate cost of a specific trajectory according to predictions
//
float calculate_cost(const Vehicle &vehicle, const Vehicle::Predictions &predictions,
                     const Vehicle::Trajectory &trajectory)
{
  Vehicle trajectory_last = trajectory.back();
  int final_lane = trajectory_last.get_tgt_lane();
  float dist = get_truncated_dist_to_vehicle(final_lane, vehicle, predictions);

  // Penalize going on a side lane
  dist -= abs(final_lane - 1) * MID_LANE_DIST_PRIORITY;

  // not to divide by zero
  dist = (dist < 0.00001) ? 0.00001 : dist;

  // calculate cost
  return (2.0 * TARGET_SPEED - trajectory_last.get_v()) / dist;
}
