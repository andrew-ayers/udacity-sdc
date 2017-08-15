#include "cost.h"
#include <math.h>
#include <iostream>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"
#include "snapshot.h"

Cost::Cost() {}
Cost::~Cost() {}

int Cost::calculate_cost(Vehicle &vehicle, vector<Snapshot> trajectories, map<int, vector <vector<int>>> predictions) {
  TrajectoryData trajectory_data = this->get_helper_data(vehicle, trajectories, predictions);

  int cost = 0;

  cost += this->inefficiency_cost(vehicle, trajectories, predictions, trajectory_data);
  cost += this->collision_cost(vehicle, trajectories, predictions, trajectory_data);
  //cost += this->buffer_cost(vehicle, trajectories, predictions, trajectory_data);
  //cout << "****************************" << endl;

  return cost;
}

TrajectoryData Cost::get_helper_data(Vehicle vehicle, vector<Snapshot> trajectories, map<int, vector <vector<int>>> predictions) {
  TrajectoryData trajectory_data;

  vector<Snapshot> t = trajectories;

  Snapshot current_snapshot = t[0];
  Snapshot first = t[1];
  Snapshot last = t[t.size() - 1];

  int dt = trajectories.size();

  trajectory_data.proposed_lane = first.lane;

  trajectory_data.avg_speed = (last.s - current_snapshot.s) / dt;

  // initialize a bunch of variables
  vector<int> accels;

  int closest_approach = 9999999;
  bool collides = false;

  map<int, vector<vector<int>>> filtered = this->filter_predictions_by_lane(predictions, trajectory_data.proposed_lane);

  for (int i = 1; i < (PLANNING_HORIZON + 1); i++) {
    Snapshot snapshot = trajectories[i];

    accels.insert(accels.end(), snapshot.a);

    map<int, vector<vector<int>>>::iterator it = filtered.begin();

    while (it != filtered.end()) {
      int v_id = it->first;

      vector<vector<int>> v = it->second;
      vector<int> state = v[i];
      vector<int> last_state = v[i - 1];

      bool vehicle_collides = this->check_collision(snapshot, last_state[1], state[1]);
      if (vehicle_collides) {
        trajectory_data.collides = true;
        trajectory_data.collides_at = i;
      }

      int dist = abs(state[1] - snapshot.s);
      if (dist < trajectory_data.closest_approach) trajectory_data.closest_approach = dist;

      it++;
    }
  }

  // find max acceleration value
  int num_accels = accels.size();
  trajectory_data.max_acceleration = 0;
  for (int i = 0; i < num_accels; i++) {
    if (accels[i] > trajectory_data.max_acceleration) trajectory_data.max_acceleration = accels[i];
  }

  return trajectory_data;
}

map<int, vector<vector<int>>> Cost::filter_predictions_by_lane(map<int, vector <vector<int>>> predictions, int lane) {
  map<int, vector<vector<int>>> filtered;

  map<int, vector<vector<int>>>::iterator it = predictions.begin();

  while (it != predictions.end()) {
    int v_id = it->first;
    vector<vector<int>> predicted_traj = it->second;

    if (predicted_traj[0][0] == lane && v_id != -1) {
      filtered[v_id] = predicted_traj;
    }

    it++;
  }

  return filtered;
}

bool Cost::check_collision(Snapshot snapshot, int s_previous, int s_now) {
  int v_target = s_now - s_previous;

  if (s_previous < snapshot.s) return (s_now >= snapshot.s);
  if (s_previous > snapshot.s) return (s_now <= snapshot.s);
  if (s_previous == snapshot.s) return (v_target <= snapshot.v);

  return false;
}

int Cost::inefficiency_cost(Vehicle vehicle, vector<Snapshot> trajectories, map<int, vector<vector<int>>> predictions, TrajectoryData data) {
  int speed = data.avg_speed;
  int target_speed = vehicle.target_speed;
  int diff = target_speed - speed;
  int pct = diff / target_speed;
  int multiplier = pct * pct;
  return multiplier * EFFICIENCY;
}

int Cost::collision_cost(Vehicle vehicle, vector<Snapshot> trajectories, map<int, vector<vector<int>>> predictions, TrajectoryData data) {
  if (data.collides) {
    int time_til_collision = data.collides_at;
    int exponent = time_til_collision * time_til_collision;
    int mult = exp(-exponent);
    return mult * COLLISION;
  }

  return 0;
}

int Cost::buffer_cost(Vehicle vehicle, vector<Snapshot> trajectories, map<int, vector<vector<int>>> predictions, TrajectoryData data) {
  int closest = data.closest_approach;
  if (closest == 0) return 10 * DANGER;

  int timesteps_away = closest / data.avg_speed;
  if (timesteps_away > DESIRED_BUFFER) return 0;

  int multiplier = 1 - pow((timesteps_away / DESIRED_BUFFER), 2);

  return multiplier * DANGER;
}
