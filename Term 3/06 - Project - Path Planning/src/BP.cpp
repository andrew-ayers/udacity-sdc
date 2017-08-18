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

  vector<double> ego_config = {this->road.speed_limit, static_cast<double>(this->road.num_lanes), static_cast<double>(this->EGO_MAX_ACCEL)};

  this->road.add_ego(this->EGO_START_LANE, 0, ego_config);
}

BP::~BP() {}

void BP::advance(vector<vector<double>> sensor_fusion, double ego_s) {
  this->road.populate_traffic(sensor_fusion);

  if (ticks++ < max_ticks) return;

  ticks = 0;

  this->road.advance(ego_s);  // pass in simulator reported s-value to sync up ego s-value
}

Vehicle BP::get_ego() {
  return this->road.get_ego();
}
