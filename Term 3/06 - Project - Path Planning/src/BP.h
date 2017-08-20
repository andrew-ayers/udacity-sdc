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

 public:
  BP();

  virtual ~BP();

  void advance(vector<vector<double>> sensor_fusion, double ego_s = 0);

  Vehicle get_ego();
};
#endif /* BEHAVIOR_H */
