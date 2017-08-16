#include "PP.h"
#include <cmath>
#include <vector>
#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"
#include "helper.h"
#include "BP.h"

//
// PP (path planner) class definition implementation.
//
PP::PP(
  vector<double> map_waypoints_x,
  vector<double> map_waypoints_y,
  vector<double> map_waypoints_s,
  vector<double> map_waypoints_dx,
  vector<double> map_waypoints_dy
) {
  this->map_waypoints_x = map_waypoints_x;
  this->map_waypoints_y = map_waypoints_y;
  this->map_waypoints_s = map_waypoints_s;
  this->map_waypoints_dx = map_waypoints_dx;
  this->map_waypoints_dy = map_waypoints_dy;
}

PP::~PP() {}

vector<double> PP::Solve(vector<double> car_data,
                         vector<vector<double>> sensor_fusion,
                         vector<double> previous_path_x,
                         vector<double> previous_path_y,
                         vector<double> end_path_sd) {
  // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
  this->car_x = car_data[0];
  this->car_y = car_data[1];
  this->car_s = car_data[2];
  this->car_d = car_data[3];
  this->car_yaw = car_data[4];
  this->car_speed = car_data[5];

  this->sensor_fusion = sensor_fusion;
  this->previous_path_x = previous_path_x;
  this->previous_path_y = previous_path_y;
  this->previous_path_size = this->previous_path_x.size();
  this->end_path_sd = end_path_sd;

  if (this->previous_path_size > 0) this->car_s = this->end_path_sd[0];

  this->bplanner.advance(sensor_fusion, this->car_s);  // pass in simulator reported s-value to sync up ego s-value

  // Convert the solution to a vector
  vector<double> path = this->GeneratePath();

  return path;
}

vector<double> PP::GeneratePath() {
  vector<double> path;

  Vehicle ego = this->bplanner.get_ego();

  // ego.lane;
  // ego.s;
  // ego.v;
  // ego.a;
  // ego.target_speed;
  // ego.state;

  this->car_ref_vel = static_cast<double>(ego.v);
  this->car_lane = ego.lane;

  cout << "Ego: S = " << ego.s
       << ", Lane = " << ego.lane
       << ", Accel = " << ego.a
       << ", Velocity = " << ego.v
       << ", State = " << ego.state
       << endl;

  return this->GenerateKeepLane();
}

// Generate a path based on nearby waypoints to keep our
// car in it's current lane
vector<double> PP::GenerateKeepLane() {
  // interleaved (x,y) path points we will return for path plan
  vector<double> path;

  // create a list of widely spaced waypoints, evenly spaced at 30m
  // later we will interpolate these waypoints with a spline and fill
  // it in with more points that control speed
  vector<double> ptsx;
  vector<double> ptsy;

  // reference x, y, and yaw states
  // either we will reference the starting point as where the car is
  // or at the previous path's end point
  double ref_x = this->car_x;
  double ref_y = this->car_y;
  double ref_yaw = deg2rad(this->car_yaw);

  // if previous path is almost empty, use the car as starting reference
  if (this->previous_path_size < 2) {
    double prev_car_x = this->car_x - cos(deg2rad(this->car_yaw));
    double prev_car_y = this->car_y - sin(deg2rad(this->car_yaw));

    // use two points that make the path tangent to the car
    ptsx.push_back(prev_car_x);
    ptsx.push_back(this->car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(this->car_y);
  } else {  // use the previous path's end point as the starting reference
    // redefine reference state as previous path end point
    ref_x = this->previous_path_x[this->previous_path_size - 1];
    ref_y = this->previous_path_y[this->previous_path_size - 1];

    // and second to last end point
    double ref_x_prev = this->previous_path_x[this->previous_path_size - 2];
    double ref_y_prev = this->previous_path_y[this->previous_path_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // use two points that make the path tangent to the previous path's
    // end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // use frenet to evenly add 30m spaced points ahead of starting reference
  int lane = getLaneFrenet(this->car_lane, 0);
  vector<double> next_wp0 = getXY(this->car_s + 30, lane, this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
  vector<double> next_wp1 = getXY(this->car_s + 60, lane, this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
  vector<double> next_wp2 = getXY(this->car_s + 90, lane, this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  // NOTE: these 5 (x,y) points are our "spline anchor points"; the
  // spline will be fitted to these, but later we will generate a set
  // of "in between points" on the spline for following

  // translate and rotate all points to the car's
  // local coordinate system; set the origin of points
  // to 0,0 (translate) and the rotation to 0 degrees
  for (int i = 0; i < ptsx.size(); i++) {
    // shift car reference angle to 0 degrees
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  // create a spline
  tk::spline s;

  // set (x,y) points to the spline
  s.set_points(ptsx, ptsy);

  // fill in with the remainder of the previous path that the
  // simulator reports (dots not consumed pac-man like!)
  for (int i = 0; i < this->previous_path_size; i++) {
    path.push_back(this->previous_path_x[i]);
    path.push_back(this->previous_path_y[i]);
  }

  // calculate how to break up the spline points so that we travel
  // at our desired reference velocity
  double target_x = 30.0;  // horizon
  double target_y = s(target_x);
  double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

  double x_add_on = 0;  // start at origin (local car coordinates)

  // fill up the rest of our path plan after filling it with
  // the previous points, here we will always output 50 points
  // Note: 2.24 is our conversion factor, 1 m/s = 2.24 mph
  for (int i = 1; i <= 50 - this->previous_path_size; i++) {
    double N = (target_dist / (.02 * this->car_ref_vel / 2.24));
    double x_point = x_add_on + (target_x) / N;
    double y_point = s(x_point);

    x_add_on = x_point;  // shift to next point

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate and translate points back to global coordinates
    // to reverse earlier translate/rotate to local car coordinates
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    path.push_back(x_point);
    path.push_back(y_point);
  }

  return path;
}
