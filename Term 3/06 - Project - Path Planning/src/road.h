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
#include "config.h"
#include "vehicle.h"

using namespace std;

class Road {
 public:
  // "prime the pump" so that first tick will run
  // the behavior planner for ego
  int ticks = MAX_TICKS;

  map<int, Vehicle> vehicles;

  int ego_key = -1;  // identifier for ego vehicle in vehicles map

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

  void advance(double ego_s = 0);

  void add_ego(int lane_num, double s);
};
#endif /* ROAD_H */
