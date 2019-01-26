# Model Documentation

The function, which performs path planning is called `plan_path` and is located in [src/path_planner.cpp](https://github.com/sand1k/CarND-Path-Planning-Project/blob/master/src/path_planner.cpp), it is called on each iteration of planning.

To predict the trajectories of other vehicle and plan movement of the car I created Vehicle class, which is located in files [src/vehicle.h](https://github.com/sand1k/CarND-Path-Planning-Project/blob/master/src/vehicle.h) and [src/vehicle.cpp](https://github.com/sand1k/CarND-Path-Planning-Project/blob/master/src/vehicle.cpp).


The algorithm is the following:

1. Create a Vehicle object, which represents the car (see line 32 of `path_planner.cpp`):

```
Vehicle ego_vehicle(g_target_lane, car_s, car_d, g_velocity, g_acceleration, g_state, 0);
```
Variables `g_target_lane`, `g_velocity`, `g_acceleration` and `g_state` are global, so they are saved between iterations in order not to loose current state of the vehicle.

2. Create Vehicle objects representing other cars on the road (initialize them from `sensor_fusion` array) and predict their positions at the end of path which was already predicted:

```
Vehicle ego_vehicle(g_target_lane, car_s, car_d, g_velocity, g_acceleration, g_state, 0);

Vehicle::Predictions predictions;
for (vector<vector<double>>::const_iterator it = sensor_fusion.begin(); it != sensor_fusion.cend(); ++it)
{
  ...
  Vehicle vehicle(l, s, d, sqrt(vx * vx + vy * vy), 0, Vehicle::State::KL, v_id);
  predictions[v_id] = vehicle.generate_predictions(prev_size * 0.02);
}

```
I use "keep lane" model for other cars for simplicity assuming that they move with constant speed and do not change lines.

3. Choose trajectory of the car given the predicted positions and velocity of other cars.

```
vector<Vehicle> trajectory = ego_vehicle.choose_next_state(predictions);
```
This is a complex step. Inside `choose_next_state` function, at first, I generate possible next states of the car given the current lane. For example, for lane №1 (middle lane) available next states are: LCL (left change left), LCR (left change right) and KL (keep lane). Secondly, I predict trajectories for each possible state (this is implemented in function `Vehicle::generate_trajectory`). For LCL and LCR states I check if the trajectory will lead for collision and return empty trajectory in this case. The function `vector<float> Vehicle::get_kinematics(const Vehicle::Predictions &predictions, int new_lane)` is used to plan car movement. It checks if there is a car ahead and chooses appropriate speed verifying that acceleration won't exceed limits. After all possible trajectories are generated I compare them using cost function, which is the following:

```
cost = (2 * TARGET_SPEED - <speed on trajectory>) / (<distance to the next vehicle on target lane> - <side lane penalty>)
```
The main idea is to choose the trajectory with fastest speed and lowest traffic. The interesting thing here is to penalize for moving in the side lane in case other parameters are almost the same. This forces the car too choose the middle lane in similar conditions, which leads to more room for maneuvers.
This step selects car position and other parameters after 1 second of movement.

4. Generate new points of car trajectory for each 0.02 second
Here I use the same approach as in "Q&A section". I use previous points of the path and point from step №3 to create a spline. And the use its points to generate final trajectory. One change here is that I consider car acceleration to plan car movement more accurately.
