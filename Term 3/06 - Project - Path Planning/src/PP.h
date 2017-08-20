#ifndef PLANNER_H
#define PLANNER_H

#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "BP.h"

using namespace std;

class PP {
 private:
  BP bplanner = BP();

  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  int car_lane;

  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;

  double car_ref_vel = 0.0;

  vector<vector<double>> sensor_fusion;
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  int previous_path_size;
  vector<double> end_path_sd;

 public:
  PP(
    vector<double> map_waypoints_x,
    vector<double> map_waypoints_y,
    vector<double> map_waypoints_s,
    vector<double> map_waypoints_dx,
    vector<double> map_waypoints_dy);

  virtual ~PP();

  // Solve for the path plan, given a bunch of information about the car and enviroment
  // Returns the path
  vector<double> Solve(vector<double> car_data,
                       vector<vector<double>> sensor_fusion,
                       vector<double> previous_path_x,
                       vector<double> previous_path_y,
                       vector<double> end_path_sd);

  vector<double> GeneratePath();

  vector<double> GenerateKeepLane();
};

#endif /* PLANNER_H */
