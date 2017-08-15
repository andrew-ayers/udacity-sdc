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
    if (this_oah_id != this->ego_key) {
      i = this->vehicles.erase(i);
    } else {
      i++;
    }
  }

  // repopulate vehicle list with sensor data
  for (int i = 0; i < sensor_fusion.size(); i++) {
    int oah_id = static_cast<int>(sensor_fusion[i][0]);  // unique id for car

    int oah_x = static_cast<int>(sensor_fusion[i][1]);   // x-position global map coords
    int oah_y = static_cast<int>(sensor_fusion[i][2]);   // y-position global map coords
    double oah_vx = sensor_fusion[i][3];                 // x-component of car's velocity
    double oah_vy = sensor_fusion[i][4];                 // y-component of car's velocity
    int oah_s = static_cast<int>(sensor_fusion[i][5]);   // how far down the road is the car?
    int oah_d = static_cast<int>(sensor_fusion[i][6]);   // what lane is the car in?

    int oah_lane = getLaneNumber(oah_d);

    // compute the magnitude of the velocity (distance formula)
    double oah_speed = sqrt(oah_vx * oah_vx + oah_vy * oah_vy);

    Vehicle vehicle = Vehicle(oah_lane, oah_s, oah_speed);
    vehicle.state = "CS";
    this->vehicles.insert(std::pair<int, Vehicle>(oah_id, vehicle));
  }
}

void Road::advance() {
  map<int, vector<vector<int>>> predictions;

  map<int, Vehicle>::iterator it = this->vehicles.begin();
  while (it != this->vehicles.end()) {
      int v_id = it->first;
      vector<vector<int>> preds = it->second.generate_predictions(30);//5
      predictions[v_id] = preds;
      it++;
  }

  it = this->vehicles.begin();
  while (it != this->vehicles.end()) {
    int v_id = it->first;

    if (v_id == ego_key) {
      it->second.update_state(predictions);
      it->second.realize_state(predictions);
    }

    it->second.increment();  // default dt = 0.02s

    if (v_id == ego_key) {
      cout << "EGO-A: " << it->second.a << ", EGO-V: " << it->second.v << endl;
    }

    it++;
  }
}

void Road::add_ego(int lane_num, int s, vector<int> config_data) {
  Vehicle ego = Vehicle(lane_num, s, 0, 0);

  ego.configure(config_data);

  ego.state = "KL";

  this->vehicles.insert(std::pair<int, Vehicle>(ego_key, ego));
}
