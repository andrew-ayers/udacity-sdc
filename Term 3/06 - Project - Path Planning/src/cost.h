#ifndef COST_H
#define COST_H
#include <math.h>
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "snapshot.h"

using namespace std;

struct TrajectoryData {
  int proposed_lane;
  double avg_speed;
  double max_acceleration;
  double closest_approach;
  bool collides;
  double collides_at;
};

// priority levels for costs
const int COLLISION  = pow(10, 6);
const int DANGER     = pow(10, 5);
const int COMFORT    = pow(10, 4);
const int EFFICIENCY = pow(10, 2);

const int PLANNING_HORIZON = 2;
const int DESIRED_BUFFER = 2;  // timesteps

class Vehicle;

class Cost {
 public:
  /**
   * Constructor
   */
  Cost();

  /**
   * Destructor
   */
  virtual ~Cost();

  int calculate_cost(const Vehicle &vehicle, vector<Snapshot> trajectories, map<int, vector<vector<int>>> predictions);

  TrajectoryData get_helper_data(Vehicle vehicle, vector<Snapshot> trajectories, map<int, vector<vector<int>>> predictions);

  map<int, vector<vector<int>>> filter_predictions_by_lane(map<int, vector<vector<int>>> predictions, int lane);

  bool check_collision(Snapshot snapshot, double s_previous, double s_now);

  int inefficiency_cost(Vehicle vehicle, vector<Snapshot> trajectories, map<int, vector<vector<int>>> predictions, TrajectoryData data);
  int collision_cost(Vehicle vehicle, vector<Snapshot> trajectories, map<int, vector<vector<int>>> predictions, TrajectoryData data);
  int buffer_cost(Vehicle vehicle, vector<Snapshot> trajectories, map<int, vector<vector<int>>> predictions, TrajectoryData data);
};
#endif /* COST_H */
