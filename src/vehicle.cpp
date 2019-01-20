#include "vehicle.h"
#include <assert.h>


int Vehicle::lane_direction_[4] = {-1, 1, -1, 1};
const char *Vehicle::state_names_[6] =
{
  "PLCL",
  "PLCR",
  "LCL",
  "LCR",
  "KL",
  "CS"
};

static float lane_center(int lane)
{
  return 2.0 + 4.0 * lane;
}


Vehicle::Vehicle(double s, double d, double speed, double accel, Vehicle::State state, int id)
{
  state_ = state;
  lane_ = d / 4;
  s_ = s;
  d_ = d;
  v_ = speed;
  a_ = accel;
  id_ = id;
}

vector<Vehicle> Vehicle::choose_next_state(int tgt_lane, const Predictions &predictions)
{
  vector<State> states = successor_states(tgt_lane);
  float cost;
  vector<float> costs;
  vector<vector<Vehicle>> final_trajectories;

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

  float next_s = position_at(dt);
  float next_v = v_;
  /*if (i < horizon - 1)
  {
    next_v = position_at(i + 1) - s_;
  }*/
  float next_d = lane_center(lane_);
  predictions.push_back(Vehicle(next_s, next_d, next_v, 0.0, KL, id_));

  return predictions;
}

vector<Vehicle::State> Vehicle::successor_states(int tgt_lane)
{
  vector<State> states;
  if ((state_ == LCL || state_ == LCR) && lane_ != tgt_lane)
  {
    states.push_back(state_);
    return states;
  }

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


//
// Given a possible next state, generate the appropriate trajectory to realize the next state.
//
Vehicle::Trajectory Vehicle::generate_trajectory(Vehicle::State state, const Vehicle::Predictions &predictions)
{
  Trajectory trajectory;
  /*if (state == CS)
  {
    trajectory = constant_speed_trajectory();
  }
  else */
  if (state == KL)
  {
    trajectory = keep_lane_trajectory(predictions);
  }
  else if (state == LCL || state == LCR)
  {
    trajectory = lane_change_trajectory(state, predictions);
  }
  else if (state == PLCL || state == LCR)
  {
    trajectory = prep_lane_change_trajectory(state, predictions);
  }
  return trajectory;
}

float Vehicle::position_at(float t)
{
    return s_ + v_ * t + a_ * t * t / 2.0;
}

//
// Generate a constant speed trajectory.
//
/*Vehicle::Trajectory Vehicle::constant_speed_trajectory()
{
  float next_pos = position_at(1);
  Trajectory trajectory =
  {
    Vehicle(s_, d_, v_, state_),
    Vehicle(next_pos, lane_center(lane_), v_, 0.0, state_)
  };
  return trajectory;
}*/

//
// Generate a keep lane trajectory.
//
Vehicle::Trajectory Vehicle::keep_lane_trajectory(const Predictions &predictions)
{
  Trajectory trajectory = {Vehicle(s_, d_, v_, a_, state_, 0)};
  vector<float> kinematics = get_kinematics(predictions, lane_);
  float new_s = kinematics[0];
  float new_d = kinematics[1];
  float new_v = kinematics[2];
  float new_a = kinematics[3];
  trajectory.push_back(Vehicle(new_s, new_d, new_v, new_a, KL, 0));
  return trajectory;
}

//
// Generate a lane change trajectory.
//
Vehicle::Trajectory Vehicle::lane_change_trajectory(State state, const Predictions &predictions)
{
  int new_lane = lane_ + lane_direction_[state];
  Trajectory trajectory;
  Vehicle next_lane_vehicle;
  //Check if a lane change is possible (check if another vehicle occupies that spot).
  for (Predictions::const_iterator it = predictions.cbegin(); it != predictions.cend(); ++it)
  {
    next_lane_vehicle = it->second[0];
    if (next_lane_vehicle.lane_ == new_lane && next_lane_vehicle.s_ > s_ - 7.0 && next_lane_vehicle.s_ < s_ + 7.0)
    {
      //If lane change is not possible, return empty trajectory.
      return trajectory;
    }
  }
  trajectory.push_back(Vehicle(s_, d_, v_, a_, state_, 0));
  vector<float> kinematics = get_kinematics(predictions, new_lane);
  trajectory.push_back(Vehicle(kinematics[0], kinematics[1], kinematics[2], kinematics[3], state, 0));
  return trajectory;
}

//
// Generate a trajectory preparing for a lane change.
//
Vehicle::Trajectory Vehicle::prep_lane_change_trajectory(State state, const Predictions &predictions)
{
  float new_s;
  float new_d;
  float new_v;
  float new_a;
  const Vehicle *vehicle_behind;
  int new_lane = lane_ + lane_direction_[state];
  Trajectory trajectory = {Vehicle(s_, d_, v_, a_, state_, 0)};
  vector<float> curr_lane_new_kinematics = get_kinematics(predictions, lane_);

  if (get_vehicle_behind(predictions, lane_, &vehicle_behind)) {
    //Keep speed of current lane so as not to collide with car behind.
    new_s = curr_lane_new_kinematics[0];
    new_d = curr_lane_new_kinematics[1];
    new_v = curr_lane_new_kinematics[2];
    new_a = curr_lane_new_kinematics[3];
  }
  else
  {
    vector<float> best_kinematics;
    vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
    //Choose kinematics with lowest velocity.
    if (next_lane_new_kinematics[2] < curr_lane_new_kinematics[2])
    {
      best_kinematics = next_lane_new_kinematics;
    }
    else
    {
      best_kinematics = curr_lane_new_kinematics;
    }
    new_s = best_kinematics[0];
    new_d = d_;
    new_v = best_kinematics[2];
    new_a = best_kinematics[3];
  }

  trajectory.push_back(Vehicle(new_s, new_d, new_v, new_a, state, 0));
  return trajectory;

}

//
// Gets next timestep kinematics (s, d, velocity)
// for a given lane. Tries to choose the maximum velocity and acceleration,
// given other vehicle positions and accel/velocity constraints.
//
vector<float> Vehicle::get_kinematics(const Vehicle::Predictions &predictions, int new_lane)
{
  const double max_acceleration = 3.0;
  float max_velocity_accel_limit = max_acceleration + v_;
  float min_velocity_accel_limit = -max_acceleration + v_;
  float new_position;
  float new_velocity;
  float new_accel;
  const Vehicle *veh_ahe;
  const Vehicle *veh_beh;

  if (get_vehicle_ahead(predictions, new_lane, &veh_ahe))
  {
    printf("  vehicle ahead: #%d dist=%f\n", veh_ahe->id_, veh_ahe->s_ - s_);

    /*if (get_vehicle_behind(predictions, new_lane, &veh_beh))
    {
      printf("vehicle #%d behind: new_lane=%d s=%f d=%f\n", veh_beh->id_, veh_beh->lane_, veh_beh->s_, veh_beh->d_);
      //must travel at the speed of traffic, regardless of preferred buffer
      new_velocity = veh_ahe->v_;
    }
    else*/
    {
      float max_velocity_in_front = (veh_ahe->s_ - s_ - PREFERRED_BUFFER) + veh_ahe->v_;
      assert(max_velocity_in_front > 0);
      new_velocity = min(max_velocity_in_front, max_velocity_accel_limit);
      new_velocity = min(new_velocity, TARGET_SPEED);
      new_velocity = max(min_velocity_accel_limit, new_velocity);
    }
  }
  else
  {
    printf("  vehicle ahead: -\n");
    new_velocity = min(max_velocity_accel_limit, TARGET_SPEED);
  }

  new_accel = new_velocity - v_; //Equation: (v_1 - v_0)/t = acceleration
  new_position = s_ + new_velocity + new_accel / 2.0;
  cout << "s=" << new_position << " d=" << lane_center(lane_) << " v=" << new_velocity << " a=" << new_accel << endl;
  return {new_position, lane_center(new_lane), new_velocity, new_accel};
}

//
// Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
// rVehicle is updated if a vehicle is found.
//
bool Vehicle::get_vehicle_behind(const Vehicle::Predictions &predictions, int lane, const Vehicle **rVehicle) const
{
  int max_s = -1;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (Predictions::const_iterator it = predictions.cbegin(); it != predictions.cend(); ++it)
  {
    temp_vehicle = it->second.at(0);
    if (temp_vehicle.lane_ == lane && temp_vehicle.s_ < s_ && temp_vehicle.s_ > max_s)
    {
      max_s = temp_vehicle.s_;
      (*rVehicle) = &it->second[0];
      found_vehicle = true;
    }
  }
  return found_vehicle;
}

//
// Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
// rVehicle is updated if a vehicle is found.
///
bool Vehicle::get_vehicle_ahead(const Vehicle::Predictions &predictions, int lane, const Vehicle **rVehicle) const
{
  float min_s = FLT_MAX;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (Predictions::const_iterator it = predictions.cbegin(); it != predictions.cend(); ++it)
  {
    temp_vehicle = it->second[0];
    if (temp_vehicle.lane_ == lane && temp_vehicle.s_ > s_ && temp_vehicle.s_ < min_s)
    {
      min_s = temp_vehicle.s_;
      (*rVehicle) = &it->second[0];
      found_vehicle = true;
    }
  }
  return found_vehicle;
}

static float get_dist_to_vehicle(int lane, const Vehicle &vehicle, const Vehicle::Predictions &predictions)
{
  const Vehicle *veh_ahe;
  if (vehicle.get_vehicle_ahead(predictions, lane, &veh_ahe))
  {
    return veh_ahe->get_s() - vehicle.get_s();
  }
  else
  {
    return FLT_MAX;
  }
}

float calculate_cost(const Vehicle &vehicle, const Vehicle::Predictions &predictions,
                     const Vehicle::Trajectory &trajectory)
{
  Vehicle trajectory_last = trajectory[1];
  int final_lane = trajectory_last.get_lane();
  float cost = (2.0 * TARGET_SPEED - trajectory_last.get_v()) / get_dist_to_vehicle(final_lane, vehicle, predictions);

  printf("%s fin_lane=%d lane_spd=%f cost=%f\n", Vehicle::state_names_[trajectory_last.get_state()],
         final_lane, trajectory_last.get_v(), cost);

  return cost;
}
