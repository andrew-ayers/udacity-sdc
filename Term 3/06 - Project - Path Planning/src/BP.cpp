#include "BP.h"
#include <math.h>
#include <vector>
#include <iostream>
#include <string>
#include "road.h"

//
// BP (behavior planner) class definition implementation.
//
BP::BP() {
  this->road = Road();

  vector<int> ego_config = {this->road.speed_limit, this->road.num_lanes, this->EGO_MAX_ACCEL};

  this->road.add_ego(this->EGO_START_LANE, 0, ego_config);
}

BP::~BP() {}

void BP::advance(vector<vector<double>> sensor_fusion) {
  this->road.populate_traffic(sensor_fusion);

  this->road.advance();
}

Vehicle BP::get_ego() {
  return this->road.get_ego();
}
