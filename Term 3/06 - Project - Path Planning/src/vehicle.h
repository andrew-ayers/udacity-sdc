#ifndef VEHICLE_H
#define VEHICLE_H
#include <math.h>
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"
#include "snapshot.h"

using namespace std;

class Vehicle {
 private:
  Cost cost;

 public:
  struct collider {
    bool collision;  // is there a collision?
    int time;        // time collision happens
  };

  struct snapshot {
    int lane;
    int s;
    int v;
    int a;
    string state;
  };

  int preferred_buffer = 100;  // impacts "keep lane" behavior.

  int lane;
  int s;
  double v;
  double a;
  double target_speed;
  int lanes_available;
  double max_acceleration;

  string state;

  /**
   * Constructor
   */
  Vehicle(int lane, int s, double v, double a = 0);

  /**
   * Destructor
   */
  virtual ~Vehicle();

  void update(int lane, int s, double v, double a);

  void update_state(map<int, vector <vector<int>>> predictions);

  string get_next_state(map<int, vector <vector<int>>> predictions);

  string min_cost_state(vector<string> states, vector<int> costs);

  vector<Snapshot> trajectories_for_state(string state, map<int, vector <vector<int>>> predictions, int horizon = 5);

  void configure(vector<int> road_data);

  string display();

  void increment(int dt = 1);

  vector<double> state_at(int t);

  bool collides_with(Vehicle other, int at_time);

  collider will_collide_with(Vehicle other, int timesteps);

  void realize_state(map<int, vector<vector<int>>> predictions);

  void realize_constant_speed();

  double _max_accel_for_lane(map<int, vector<vector<int>>> predictions, int lane, int s);

  void realize_keep_lane(map<int, vector< vector<int>>> predictions);

  void realize_lane_change(map<int, vector<vector<int>>> predictions, string direction);

  void realize_prep_lane_change(map<int, vector<vector<int>>> predictions, string direction);

  vector<vector<int>> generate_predictions(int horizon = 10);
};
#endif /* VEHICLE_H */
