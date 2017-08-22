[//]: # (Image References)

[image1]: ./images/self_driving_car_nanodegree_program.png "Title Image"

# **<center>Term 3 - Project 1: Path Planning - Part 2: Reflection</center>**

<center>![alt text][image1]</center>

### Reflection on the Path Planner Process

The following is a brief rundown of each of the elements involved in the execution of the path planner and how the path to follow is generated. The explanations are presented in the same order as they were previously listed above in the code inclusion section at the beginning of this README file.

#### 1. Initial execution starts in main.cpp

Most of main.cpp is involved in incidental functioning, and wasn't altered much for the project.

The code first reads in the waypoint data from the ""/data/highway_map.csv" file, which essentially is a set of waypoint data information which follows the centerline of the simulated highway road.

The data consists of a series of data points, each as a global map x/y Cartesian coordinate pair, along with a frenet coordinate pair of s (distance along the road) and d (distance from centerline). The "d" portion of this coordinate pair is curious in that it is given as an x/y unit vector pair "dx" and "dy". This vector pair is later converted into a unit vector which can be used to calculate distance from the centerline.

After the data is read, the path planner class (PP.cpp) is instantiated and initialized (main.cpp, line 70):

```c
PP planner = PP(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
```

The various waypoint data is passed into the planner for later usage. After this is done, the main simulation loop begins, getting data from the simulator and sending it to the planner.

The data from the simulator is comprised of a number of different pieces:

* The ego vehicle's location (x/y, s/d), angle (yaw), and speed
* The ego vehicle's previous path information
* Sensor fusion data about other cars around the vehicle in the right-hand lanes

These values are then passed to the path planner "solver" method (main.cpp, lines 114-129):

```c
vector<double> car_data = {
  car_x,
  car_y,
  car_s,
  car_d,
  car_yaw,
  car_speed
};

vector<double> end_path_sd = {
  end_path_s,
  end_path_d
};

// Run the solver to calculate path plan
vector<double> path_plan = planner.Solve(car_data, sensor_fusion, previous_path_x, previous_path_y, end_path_sd);
```

The output from the planner is a vector of x and y Cartesian coordinates, representing the path the ego vehicle is to follow, which are passed back to the simulator (main.cpp, lines 133-146):

```c
for (size_t i = 0; i < path_plan.size(); i++) {
  if (i % 2 == 0) {
    next_x_vals.push_back(path_plan[i]);
  } else {
    next_y_vals.push_back(path_plan[i]);
  }
}

msgJson["next_x"] = next_x_vals;
msgJson["next_y"] = next_y_vals;

auto msg = "42[\"control\","+ msgJson.dump()+"]";

ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
```

#### 2. Path planner configuration header (config.h)

The configuration file is included and utilized by almost every other part of the process. Its purpose is to create a general set of global constants to be referenced within the code, with a consistent naming convention and structure.

This was done so that I could easily tweak and adjust the various parameters involved in the processing to attempt to effect a desired outcome, or to correct other issues. Prior to implementing this, altering these values was difficult and required the careful adjustment over multiple files.

Now, a single file can be altered to affect the way the entire system executes. A sample portion of this file looks like (config.h, lines 6-13):

```c
// general settings (DO NOT TOUCH)
const bool DEBUG = true;

const double MPH_TO_MPS = 2.24;     // MPH to meters-per-second, 2.24mph -> 1 m/s
const double SPLINE_SPACING = 30;   // 30 meters between spline segments
const int MAX_TICKS = 30;           // number of simulator "ticks" before behavior planner processing for ego
const double SECS_PER_TICK = 0.02;  // number of elapsed seconds per "tick" of simulator
const int FILLER_DIST = 50;         // 50 meters
```

#### 3. Execution of the path planner class (PP.cpp)

The path planner class consists of a few basic methods. First is the constructor, which is responsible for taking waypoint information and storing it in local variables for later reference. This was described somewhat above and illustrated in regards to how main.cpp worked, so I won't repeat it here.

Next is the "Solve()" method; this method takes in car and sensor fusion data (as also described above in the explanation of main.cpp), saves a lot of it local to the class, and immediately passes some of it to the behavior planner (PP.cpp, line 52):

```c
this->bplanner.advance(sensor_fusion, this->car_s);  // pass in simulator reported s-value to sync up ego s-value
```

Notice that the only information we pass into the planner is the sensor fusion data and the ego vehicle's frenet s-coordinate value. We need the sensor fusion data obviously, because it contains information about the state of the other vehicles on the road. But we only care about the ego vehicle's current s-value, because all the other values for the vehicle will be computed by the planner, but it is very important that we know the current s-value, for reasons that will be explained later (although the hint in the comment should be a big clue).

Once the behavior planner is done processing, all the information needed for generation of the path will be available. At this point, the path generation method is called and the data returned to main.cpp for output back to the simulator (PP.cpp, line 55):

```c
vector<double> path = this->GeneratePath();
```

PP::GeneratePath() first gets the information about the ego vehicle; what lane it is supposed to be in, its s-value, its velocity and acceleration, its target speed, and its state. Most of this data is meant for debugging purposes (PP.cpp, lines 76-85):

```c
if (DEBUG) {
  cout << "Ego: S = " << ego.s
       << ", Lane = " << ego.lane
       << ", Accel = " << ego.a
       << ", Velocity = " << ego.v
       << ", In Front = " << ego.in_front
       << ", At Behind = " << ego.at_behind
       << ", State = " << ego.state
       << endl;
}
```

The main path generation functionality begins with the poorly named, but necessary method, PP::GenerateKeepLane(), on line 87; the output of which is ultimately returned to main.cpp.

I won't delve too deep into what is happening in PP::GenerateKeepLane(); the code includes copius comments on its operation, plus it is identical to the code shown the video walk-through presented by Aaron Brown and David Silver in the Path Planning project overview (see https://www.youtube.com/watch?v=7sI3VHFPP0w).

The code essentially works by taking a starting reference x/y position for the car, and it's current steering angle or yaw value (PP.cpp, lines 105-107):

```c
double ref_x = this->car_x;
double ref_y = this->car_y;
double ref_yaw = deg2rad(this->car_yaw);
```

It then finds the previous position of the car, either by using a simple extrapolation based on the car's steering angle (PP.cpp, lines 110-112):

```c
if (this->previous_path_size < 2) {
  double prev_car_x = this->car_x - cos(deg2rad(this->car_yaw));
  double prev_car_y = this->car_y - sin(deg2rad(this->car_yaw));
  ...
}
```

...or by getting the information from the previous path, and finding the steering angle from those (PP.cpp, lines 122-128):

```c
ref_x = this->previous_path_x[this->previous_path_size - 1];
ref_y = this->previous_path_y[this->previous_path_size - 1];

// and second to last end point
double ref_x_prev = this->previous_path_x[this->previous_path_size - 2];
double ref_y_prev = this->previous_path_y[this->previous_path_size - 2];
ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
```

These points describe a path tangent to the car or to the previous path's endpoint respectively, forming the first point in the new path being generated. The points in both cases are pushed onto a couple of lists which are used in the path generation process.

We then find the frenet d-coordinates of the lane the ego vehicle is in, and then extrapolate a series of three x/y coordinates which follow the waypoints, but spaced 30 meters apart. These are also placed on the stack (PP.cpp, lines 140-151):

```c
int lane = getLaneFrenet(this->car_lane, 0);
vector<double> next_wp0 = getXY(this->car_s + SPLINE_SPACING, lane, this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
vector<double> next_wp1 = getXY(this->car_s + (SPLINE_SPACING * 2), lane, this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);
vector<double> next_wp2 = getXY(this->car_s + (SPLINE_SPACING * 3), lane, this->map_waypoints_s, this->map_waypoints_x, this->map_waypoints_y);

ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);
```

At this point, we have a series of five global x and y coordinates which describe a path from some given starting position to some ending position, which were given in frenet s and d coordinates, then transformed to x/y Cartesian coordinates.

At this point, we need to rotate the points to the ego vehicle's local coordinate system. This is done mainly because the technique being used to generate the curve of the path (cubic spline interpolation via spline.h) has problems when the spline runs in a vertical orientation rather than horizontal, and the x-coordinates line up. So we first rotate the points to a horizontal orientation, perform the spline interpolation, then rotate the coordinates back.

So first, we rotate, then create the spline (PP.cpp, lines 159-172):

```c
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
```

From this point, we start to do something curious. We begin to generate our path by first pushing onto our path vector information from the previous path. This path information is given to us from the simulator, and references the coordinates of the previous path that haven't been "consumed" by the ego vehicle as it travels along. You can think of the ego vehicle as being like "pac-man", eating the dots ahead of it (path coordinates), but not all the dots may be eaten before it needs to extend the path (PP.cpp, lines 176-179):

```c
for (int i = 0; i < this->previous_path_size; i++) {
  path.push_back(this->previous_path_x[i]);
  path.push_back(this->previous_path_y[i]);
}
```

We then need to calculate the proper spacing of the points so that we travel at our desired velocity. We do this by calculating a distance value based on the spline spacing of 30 meters (PP.cpp, lines 183-185):

```c
double target_x = SPLINE_SPACING;  // horizon 30 meters, 0.6s (30 * 0.02)
double target_y = s(target_x);
double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
```

We then fill in the remainder of our path points, with the x-coordinate value being adjusted based on the desired velocity and other values to adjust the spacing between the coordinates. The resulting coordinates are then rotated and transformed back to the global representation, and pushed onto the path vector (PP.cpp, lines 193-213):

```c
for (int i = 1; i <= FILLER_DIST - this->previous_path_size; i++) {
  double N = (target_dist / (SECS_PER_TICK * this->car_ref_vel / MPH_TO_MPS));
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
```

The path information is then returned up the chain, back to main.cpp for output to the simulator.

I wish to thank Udacity, Aaron Brown, and David Silver for providing this code, as it was sorely needed to help kickstart this project by myself and I would imaging others in the class as well.

#### 4. Execution of the behavior planner class (BP.cpp)

The behavior planner class consists of a constructor (which is called earlier and "secretly" by the Path Planner class header, PP.h). The constructor instantiates the "road" and adds the ego vehicle to it (BP.cpp, lines 12-16):

```c
BP::BP() {
  this->road = Road();

  this->road.add_ego(EGO_START_LANE, 0);
}
```

The BP::advance() method serves as a wrapper to the Road::advance() method; but prior to calling that method, the road is "populated" with the traffic information provided by the sensor fusion data (BP.cpp, lines 20-24):

```c
void BP::advance(vector<vector<double>> sensor_fusion, double ego_s) {
  this->road.populate_traffic(sensor_fusion);

  this->road.advance(ego_s);  // pass in simulator reported s-value to sync up ego s-value
}
```

Finally, as we saw earlier, the BP::get_ego() method is used by the path planner (PP.cpp) to get information needed for debug logging and for path generation.

#### 5. Execution of the road handler class (road.cpp)

The road handler class performs a variety of functionality related to simulating the "road" for behavior planning purposes. The "road" is conceptualized as a vector of vehicles, representing all the vehicles on the road. At regular moments, the "road" is called on to "advance" by a certain measure of time, to process a series of simulated "predictions" about where the vehicles will be, given what is known about them in the current time "slice".

First, in the Road::populate_traffic() method, the vehicle list is cleared of all vehicles <i>except</i> the ego vehicle. We don't want to lose any information about the ego vehicle, which is why it is skipped (road.cpp, lines 20-30):

```c
void Road::populate_traffic(vector<vector<double>> sensor_fusion) {
  // clear vehicle list of every vehicle except ego
  map<int, Vehicle>::iterator i = this->vehicles.begin();
  while (i != this->vehicles.end()) {
    int this_oah_id = i->first;
    if (this_oah_id == this->ego_key) {
      i++;  // skip ego
    } else {
      i = this->vehicles.erase(i);  // delete npc vehicle
    }
  }
  ...

}
```

We then repopulate the vehicle list from the sensor fusion data (road.cpp, lines 33-51):

```c
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
```

Advancing the road is accomplished by the aptly named Road::advance() method. For every timeslice from the simulator, a variety of predictions about the vehicles on the road is generated, provided the vehicle is on the track (road.cpp, lines 54-69):

```c
void Road::advance(double ego_s) {
  map<int, vector<vector<double>>> predictions;

  map<int, Vehicle>::iterator it = this->vehicles.begin();
  while (it != this->vehicles.end()) {
      int v_id = it->first;

      Vehicle vv = it->second;

      // if the lane is < 0 then vehicle is not on track, so skip
      if (vv.lane > -1) {
        predictions[v_id] = vv.generate_predictions(PREDICTIONS_HZ);
      }

      it++;
  }
  ...
}
```

I noticed this little tidbit in my testing and refactoring that sometimes there were vehicles in the sensor data that weren't on the track, as indicated by these vehicles having a lane value of -1; I'm not sure why that is, though I think it may be part of the "rolling patch of cars" method of vehicle generation - essentially, the ego vehicle is surrounded by a "patch" of generated cars, so far aft of the ego vehicle, and so far ahead of it. When the cars "fall off" the edge of the patch (by being outpaced by the ego vehicle, or going faster than the ego vehicle), they get "recycled".

There always seems to be a maximum of 12 cars at any one time on the patch, but some of those cars aren't visible, and are assigned a lane number of -1. You can see this patch effect if you zoom out from the ego vehicle and flatten the view out toward the horizon of the road; you will then see the cars in the distance pop in and out of the view as the fall off the edges (and get recycled).

After predictions, the world of vehicles is "incremented"; what this means is that each of their positions is updated according to their current data (position, acceleration, etc).

If the car is the ego vehicle, however, its information is updated at a lower rate. We first sync up the ego vehicle's s-value position, then we call the update_state() and realize_state() on the ego vehicle; these methods are only used for the ego vehicle (road.cpp, lines 71-90):

```c
it = this->vehicles.begin();
while (it != this->vehicles.end()) {
  int v_id = it->first;

  bool is_ego = (v_id == ego_key);

  ticks++;
  if (ticks > MAX_TICKS) {
    ticks = 0;
    if (is_ego) {
      it->second.s = ego_s;  // sync up ego s-value to simulator s-value
      it->second.update_state(predictions);
      it->second.realize_state(predictions);
    }
  }

  it->second.increment();

  it++;
}
```

Finally, the last method in road.cpp is Road::add_ego(), which is used by the constructor in the behavior planner class (BP.cpp) to add the ego vehicle to the list of vehicles on the "road" - aka, the road vehicle vector list. There isn't too much to note about this method; it basically instantiates a vehicle with the ego vehicle information, and inserts it into the vehicle list on the road object (road.cpp, lines 93-97):

```c
void Road::add_ego(int lane_num, double s) {
  Vehicle ego = Vehicle(lane_num, s, 0, 0, true);

  this->vehicles.insert(std::pair<int, Vehicle>(this->ego_key, ego));
}
```

#### 6. Execution of the vehicle handler class (vehicle.cpp)

* vehicle.h & vehicle.cpp - vehicle handler class: handles a variety of tasks all relating to vehicles on the simulated road, including both the ego vehicle (our vehicle the simulation is driving) and the "np" vehicles (all the other "non-player" simulated traffic vehicles around the ego vehicle)

#### 7. Execution of the cost calculations class (cost.cpp)

* cost.h & cost.cpp - cost calculations class: handles the cost calculations for the behavior planner

#### 8. Description of various helper functions (helper.cpp)

* helper.h & helper.cpp - various helper functions for finding waypoints, converting between frenet and cartesian road coordinates, and others (most functionality provided by Udacity, with a few additional custom functions added)

#### 9. Snapshot handler class (snapshot.cpp)

* snapshot.h & snapshot.cpp - a "snapshot" handler class: handles taking a single "snapshot" of the vehicle state (custom implementation based on similar functionality found within the python example)


---
### Simulation

#### 1. The vehicle must successfully drive a lap around the track

Upon running the simulation, the vehicle should navigate the highway course successfully and smoothly for a total of 4.35 miles, at speeds up to 50 mph without any incidents, similar to the example video.

#### 2. Improvements to be explored

Future improvements would be first to refactor the code a bit more to break up some of the functionality into a variety of methods called independently as needed. There are a number of areas where this would be beneficial from both a maintenance standpoint, as well as an easier means to understanding the system. For instance, a lot of the housekeeping, initialization, and preprocessing steps could each become their own functionality, whether as seperate methods within the existing MPC class, or as new classes.

Another improvement to be made would be to try additional weights to the cost calculations for other parts. Adding these weights might improve the response of the vehicle over a wider range of speeds.
