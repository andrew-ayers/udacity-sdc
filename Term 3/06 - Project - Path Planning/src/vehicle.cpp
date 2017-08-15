#include "vehicle.h"
#include <math.h>
#include <iostream>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>
#include "snapshot.h"
#include "cost.h"

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, double v, double a) {
    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = "CS";
    this->max_acceleration = -1;

    this->cost = Cost();
}

Vehicle::~Vehicle() {}

void Vehicle::update(int lane, int s, double v, double a) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
}

// TODO - Implement this method.
void Vehicle::update_state(map<int, vector <vector<int>>> predictions) {
  /*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    */
    this->state = this->get_next_state(predictions);
}

string Vehicle::get_next_state(map<int, vector <vector<int>>> predictions) {
  vector<string> states = {"KL", "PLCR", "LCR", "PLCL", "LCL"};

  if (this->lane == 0) {
    states.erase(states.begin() + 3, states.begin() + 4);
  } else if (this->lane == this->lanes_available - 1) {
    states.erase(states.begin() + 1, states.begin() + 2);
  }

  vector<string> new_states;
  vector<int> new_costs;
  string best = "KL";

  bool dothis = true;
  if (dothis) {
    for (int i = 0; i < states.size(); i++) {
      string state = states[i];

      // map<int, vector <vector<int>>> predictions_copy = predictions;

      vector<Snapshot> trajectories = this->trajectories_for_state(state, predictions);

      int cost = this->cost.calculate_cost(*this, trajectories, predictions);

      new_states.insert(new_states.end(), state);
      new_costs.insert(new_costs.end(), cost);
    }

    best = this->min_cost_state(states, new_costs);
  }

  return best;
}

string Vehicle::min_cost_state(vector<string> states, vector<int> costs) {
  string best_state = "";
  int best_cost = 999999999;

  for (int i = 0; i < states.size(); i++) {
    if (costs[i] < best_cost) {
      best_cost = costs[i];
      best_state = states[i];
    }
  }

  return best_state;
}

// NOTE: Something is wrong with this method; it seems to reset the velocity
// (and perhaps other attributes) of the ego car in some manner???
vector<Snapshot> Vehicle::trajectories_for_state(string state, map<int, vector<vector<int>>> predictions, int horizon) {
  // remember the current state of vehicle
  Snapshot current = Snapshot(this->lane, this->s, this->v, this->a, this->state);

  // build a list of trajectories
  vector<Snapshot> trajectories;

  // save the current state for the initial trajectory in the list
  trajectories.insert(trajectories.end(), current);

  for (int i = 0; i < horizon; i++) {
    // restore the state from the snapshot...
    this->lane = current.lane;
    this->s = current.s;
    this->v = current.v;
    this->a = current.a;
    // ...but pretend to be in the new proposed state
    this->state = state;

    // perform the pretended state transition
    this->realize_state(predictions);

    // update the velocity and acceleration for the next time delta
    this->increment();

    // save the trajectory
    trajectories.insert(trajectories.end(), Snapshot(this->lane, this->s, this->v, this->a, this->state));

    // need to remove first prediction for each vehicle
    map<int, vector <vector<int>>>::iterator pr = predictions.begin();
    while (pr != predictions.end()) {
      int prediction_id = pr->first;

      vector<vector<int>> prediction = pr->second;

      prediction.erase(prediction.begin());

      predictions[prediction_id] = prediction;

      pr++;
    }
  }

  // restore the vehicle's state (from the snapshot)
  this->lane = current.lane;
  this->s = current.s;
  this->v = current.v;
  this->a = current.a;
  this->state = current.state;

  return trajectories;
}

void Vehicle::configure(vector<int> road_data) {
  /*
   Called by simulator before simulation begins. Sets various
   parameters which will impact the ego vehicle.
  */

  // configuration data: speed limit, num_lanes, max_acceleration
  this->target_speed = road_data[0];
  this->lanes_available = road_data[1];
  this->max_acceleration = road_data[2];
}

string Vehicle::display() {
  ostringstream oss;

  oss << "s:    " << this->s << "\n";
  oss << "lane: " << this->lane << "\n";
  oss << "v:    " << this->v << "\n";
  oss << "a:    " << this->a << "\n";

  return oss.str();
}

void Vehicle::increment(int dt) {
  double ddt = static_cast<double>(dt) * 0.02;
  this->s += static_cast<int>(this->v * ddt);
  this->v += this->a * ddt;
}

vector<double> Vehicle::state_at(int t) {
  /*
    Predicts state of vehicle in t*0.02 seconds (assuming constant acceleration)
  */
  double dt = static_cast<double>(t) * 0.02;
  double s = this->s + this->v * dt + this->a * dt * dt / 2;
  double v = this->v + this->a * dt;
  return {static_cast<double>(this->lane), s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, int at_time) {
  /*
    Simple collision detection.
  */
  vector<double> check1 = state_at(at_time);
  vector<double> check2 = other.state_at(at_time);
  return (check1[0] == check2[0]) && (abs(check1[1] - check2[1]) <= 1);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {
  Vehicle::collider collider_temp;
  collider_temp.collision = false;
  collider_temp.time = -1;

  for (int t = 0; t < timesteps + 1; t++) {
    if (this->collides_with(other, t)) {
      collider_temp.collision = true;
      collider_temp.time = t;
      return collider_temp;
    }
  }

  return collider_temp;
}

void Vehicle::realize_state(map<int, vector<vector<int>>> predictions) {
  /*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
  */
  string state = this->state;

  if (state.compare("CS") == 0) {
     realize_constant_speed();
  } else if (state.compare("KL") == 0) {
     realize_keep_lane(predictions);
  } else if (state.compare("LCL") == 0) {
     realize_lane_change(predictions, "L");
  } else if (state.compare("LCR") == 0) {
     realize_lane_change(predictions, "R");
  } else if (state.compare("PLCL") == 0) {
     realize_prep_lane_change(predictions, "L");
  } else if (state.compare("PLCR") == 0) {
     realize_prep_lane_change(predictions, "R");
  }
}

void Vehicle::realize_constant_speed() {
  this->a = 0;
}

void Vehicle::realize_keep_lane(map<int, vector<vector<int>>> predictions) {
  this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int, vector<vector<int>>> predictions, string direction) {
  int delta = -1;

  if (direction.compare("L") == 0) {
    delta = 1;
  }

  this->lane += delta;
  int lane = this->lane;
  int s = this->s;
  this->a = _max_accel_for_lane(predictions, lane, s);
}

double Vehicle::_max_accel_for_lane(map<int, vector<vector<int>>> predictions, int lane, int s) {
  double delta_v_til_target = this->target_speed - this->v;
  double max_acc = min(this->max_acceleration, delta_v_til_target);

  //cout << "DVTT: " << delta_v_til_target << ", MAX_ACC: " << max_acc << endl;

  /*
  map<int, vector<vector<int>>>::iterator it = predictions.begin();
  vector<vector<vector<int>>> in_front;

  while (it != predictions.end()) {
    int v_id = it->first;

    vector<vector<int>> v = it->second;

    if ((v[0][0] == lane) && (v[0][1] > s)) {
      in_front.push_back(v);
    }

    it++;
  }

  if (in_front.size() > 0) {
    int min_s = 1000;

    vector<vector<int>> leading;

    for (int i = 0; i < in_front.size(); i++) {
      if ((in_front[i][0][1] - s) < min_s) {
        min_s = (in_front[i][0][1] - s);
        leading = in_front[i];
      }
    }

    double next_pos = leading.size() > 1 ? leading[1][1] : min_s;
    double my_next = s + this->v;
    double separation_next = next_pos - my_next;
    double available_room = separation_next - this->preferred_buffer;

    max_acc = min(max_acc, available_room);
  }
  cout << "DVTT: " << delta_v_til_target << ", MAX_ACC: " << max_acc << endl;
  */
  return max_acc;
}

void Vehicle::realize_prep_lane_change(map<int, vector<vector<int>>> predictions, string direction) {
  int delta = -1;
  if (direction.compare("L") == 0) {
    delta = 1;
  }

  int lane = this->lane + delta;

  map<int, vector<vector<int>>>::iterator it = predictions.begin();

  vector<vector<vector<int>>> at_behind;

  while (it != predictions.end()) {
    int v_id = it->first;

    vector<vector<int>> v = it->second;

    if ((v[0][0] == lane) && (v[0][1] <= this->s)) {
      at_behind.push_back(v);
    }

    it++;
  }

  if (at_behind.size() > 0) {
    int max_s = -1000;

    vector<vector<int>> nearest_behind;

    for (int i = 0; i < at_behind.size(); i++) {
      if ((at_behind[i][0][1]) > max_s) {
        max_s = at_behind[i][0][1];
        nearest_behind = at_behind[i];
      }
    }

    int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
    double delta_v = static_cast<double>(this->v - target_vel);
    double delta_s = static_cast<double>(this->s - nearest_behind[0][1]);

    if (delta_v != 0) {
      double time = -2 * delta_s / delta_v;
      double a;

      if (time == 0) {
        a = this->a;
      } else {
        a = delta_v / time;
      }

      if (a > this->max_acceleration) {
        a = this->max_acceleration;
      }

      if (a < -this->max_acceleration) {
        a = -this->max_acceleration;
      }

      this->a = a;
    } else {
      double my_min_acc = max(-this->max_acceleration, -delta_s);

      this->a = my_min_acc;
    }
  }
}

vector<vector<int>> Vehicle::generate_predictions(int horizon) {
  vector<vector<int>> predictions;

  for (int i = 0; i < horizon; i++) {
    vector<double> check = this->state_at(i);
    vector<int> lane_s = {static_cast<int>(check[0]), static_cast<int>(check[1])};
    predictions.push_back(lane_s);
  }

  return predictions;
}
