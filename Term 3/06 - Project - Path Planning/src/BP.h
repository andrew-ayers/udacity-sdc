#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <cmath>
#include <vector>
#include "road.h"
#include "vehicle.h"

using namespace std;

class BP {
 private:
  Road road;

  int ticks = 0;
  int max_ticks = 0;
  // At each timestep, ego can set acceleration to value between
  // -MAX_ACCEL and MAX_ACCEL
  int EGO_MAX_ACCEL = 11;  // 11 * .02 = .22, close to the 0.224 value from walkthru

  // Lane in which ego starts in
  int EGO_START_LANE = 1;

 public:
  BP();

  virtual ~BP();

  void advance(vector<vector<double>> sensor_fusion, double ego_s = 0);

  Vehicle get_ego();
};
#endif /* BEHAVIOR_H */
