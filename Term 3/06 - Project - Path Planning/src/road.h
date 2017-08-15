#ifndef ROAD_H
#define ROAD_H

#include <math.h>
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"

using namespace std;

class Road {
 public:
  int ego_key = -1;

  int num_lanes = 3;

  vector<int> lane_speeds = {50, 50, 50};

  int speed_limit = 50; //49.75

  map<int, Vehicle> vehicles;

  int vehicles_added = 0;

  /**
   * Constructor
   */
  Road();

  /**
   * Destructor
   */
  virtual ~Road();

  Vehicle get_ego();

  void populate_traffic(vector<vector<double>> sensor_fusion);

  void advance();

  void add_ego(int lane_num, int s, vector<int> config_data);
};
#endif /* ROAD_H */
