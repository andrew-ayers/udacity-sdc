#include "road.h"
#include <math.h>
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include <utility>
#include "vehicle.h"
#include "helper.h"

Road::Road() {}
Road::~Road() {}

Vehicle Road::get_ego() {
  return this->vehicles.find(this->ego_key)->second;
}

void Road::populate_traffic(vector<vector<double>> sensor_fusion) {
  // clear vehicle list of every vehicle except ego
  map<int, Vehicle>::iterator i = this->vehicles.begin();
  while (i != this->vehicles.end()) {
    int this_oah_id = i->first;
    if (this_oah_id == this->ego_key) {
      i++; // skip
    } else {
      i = this->vehicles.erase(i); // delete
    }
  }

  // repopulate vehicle list with sensor data
  for (int i = 0; i < sensor_fusion.size(); i++) {
    int oah_id = static_cast<int>(sensor_fusion[i][0]);  // unique id for car

    double oah_x = sensor_fusion[i][1];   // x-position global map coords
    double oah_y = sensor_fusion[i][2];   // y-position global map coords
    double oah_vx = sensor_fusion[i][3];  // x-component of car's velocity
    double oah_vy = sensor_fusion[i][4];  // y-component of car's velocity
    double oah_s = sensor_fusion[i][5];   // how far down the road is the car?
    double oah_d = sensor_fusion[i][6];   // what lane is the car in?

    int oah_lane = getLaneNumber(oah_d);

    // compute the magnitude of the velocity (distance formula)
    double oah_speed = sqrt(oah_vx * oah_vx + oah_vy * oah_vy);

    Vehicle vehicle = Vehicle(oah_lane, oah_s, oah_speed);
    vehicle.state = "KL";  // CS?
    this->vehicles.insert(std::pair<int, Vehicle>(oah_id, vehicle));
  }
}

void Road::advance(double ego_s) {
  map<int, vector<vector<int>>> predictions;

  map<int, Vehicle>::iterator it = this->vehicles.begin();
  while (it != this->vehicles.end()) {
      int v_id = it->first;

      Vehicle vv = it->second;

      // if the lane is < 0 then vehicle is not on track, so skip
      if (vv.lane > -1) {
        predictions[v_id] = vv.generate_predictions(100);
      }

      it++;
  }

  it = this->vehicles.begin();
  while (it != this->vehicles.end()) {
    int v_id = it->first;

    bool is_ego = (v_id == ego_key);

    if (is_ego) {
      it->second.s = ego_s;  // sync up ego s-value to simulator s-value
      it->second.update_state(predictions);
      it->second.realize_state(predictions);
    }

    // NOTE: if we are updating the ego car, don't calculate s-value, keep set value
    it->second.increment(1, is_ego);  // default dt = 0.02s

    it++;
  }
}

void Road::add_ego(int lane_num, double s, vector<double> config_data) {
  Vehicle ego = Vehicle(lane_num, s, 0, 0);

  ego.configure(config_data);

  ego.state = "KL";
  ego.is_ego = true;

  this->vehicles.insert(std::pair<int, Vehicle>(ego_key, ego));
}
