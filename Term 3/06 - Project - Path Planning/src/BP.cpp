#include "BP.h"
#include <math.h>
#include <vector>
#include <iostream>
#include <string>
#include "config.h"
#include "road.h"

//
// BP (behavior planner) class definition implementation.
//
BP::BP() {
  this->road = Road();

  this->road.add_ego(EGO_START_LANE, 0);
}

BP::~BP() {}

void BP::advance(vector<vector<double>> sensor_fusion, double ego_s) {
  this->road.populate_traffic(sensor_fusion);

  this->road.advance(ego_s);  // pass in simulator reported s-value to sync up ego s-value
}

Vehicle BP::get_ego() {
  return this->road.get_ego();
}
