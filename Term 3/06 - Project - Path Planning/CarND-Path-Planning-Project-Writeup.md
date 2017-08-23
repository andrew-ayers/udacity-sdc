# **Term 3 - Project 1: Path Planning**

![Title](http://i.imgur.com/g9COk3j.png)

---
## Goals

* Design a path planner that is able to create smooth, safe paths for the car
* The car should follow along a 3 lane highway with traffic inside its lane
* The car has to avoid hitting other cars, and pass slower moving traffic
* The car can perform this using localization, sensor fusion, and map data

---
## Specification  ([rubric](https://review.udacity.com/#!/rubrics/1020/view))

### Compilation

#### 1. Submission code is included

It should be noted that the original bare project as received from Udacity consisted of only a single file, main.cpp. Due to the complex nature of the project, and the number of interrelated yet mostly independent sub-processes involved, I decided to structure the project as a series of classes to enable better focus and understanding of each part. The submission therefore includes the following files in the /src folder which form the implementation of the Path Planner:

* main.cpp - contains the base code to instantiate and run the path planner
* config.h - header file included by other files for various constant parameter definitions
* PP.h & PP.cpp - path planner class: handles dispatch to the remainder of process, and eventual path generation
* BP.h & BP.cpp - behavior planner class: instantiator and wrapper for Road class

The following files are my attempt at converting the python example code provided in Lesson 4: Behavior Planning; it was found by another user (see https://discussions.udacity.com/t/quiz-4-16-puzzling-python-official-solution-does-not-work/335092) that the python code does not actually work for that lesson, and that the car just follows along in lane one throughout the entire run (and never reaches the goal).

After implementing this, I found the same thing happening within the context of this project, and found the user's posting to the discussion forum as a result. Ultimately, in the effort to fix this issue and others, the code in areas no longer strictly follows the original python (not to mention that the code for that quiz was more step-like than continuous, which also necessitated various changes on my part to get it to work for the path planner):

* road.h & road.cpp - road handler class: handles populating the "road" for the behavior planner, and time step advancing the behavior planner simulation
* vehicle.h & vehicle.cpp - vehicle handler class: handles a variety of tasks all relating to vehicles on the simulated road, including both the ego vehicle (our vehicle the simulation is driving) and the "np" vehicles (all the other "non-player" simulated traffic vehicles around the ego vehicle)
* cost.h & cost.cpp - cost calculations class: handles the cost calculations for the behavior planner

Finally, the following files are support library code used by the above processes:

* helper.h & helper.cpp - various helper functions for finding waypoints, converting between Frenet and Cartesian road coordinates, and others (most functionality provided by Udacity, with a few additional custom functions added)
* snapshot.h & snapshot.cpp - a "snapshot" handler class: handles taking a single "snapshot" of the vehicle state (custom implementation based on similar functionality found within the python example)

Note that the above files are listed in "process order"; that is, which part of the process is run first, then second, and so forth thru to the last portion of the total process.

The following files and folders are also in the /src folder, but are utilized as "third-party" helper libraries that aren't to be considered part of the main project submission:

* /Eigen-3.3 - Eigen C++ template library for linear algebra (see http://eigen.tuxfamily.org)
* json.hpp - JSON for Modern C++ (see https://github.com/nlohmann/json)
* spline.h - Simple cubic spline interpolation library (see http://kluge.in-chemnitz.de/opensource/spline/)

These libraries are used by the project to perform linear algebra operations, communicate data to and from the Udacity vehicle simulation, and to assist in the generation of splines for the path planner.

#### 2. Submission code should compile

First note that the "CMakeLists.txt" file (located in the root folder of the project submission) has been slightly modified to support the compilation of the multiple files listed in part 1 above.

To compile all of the file, open a terminal shell window in the /build folder of the project repository. Execute the following:

```bash
$ clear; cmake .. && make
```

The code should compile successfully and be ready to use.

#### 3. Compiled submssion code is usable

First, open a terminal shell window in the /build folder of the project repository. The vehicle can be made to operate by executing:

```bash
$ ./path_planning
```

You should get a message stating:

```
Listening to port 4567
```

...indicating that the path planner is waiting for the simulator to be started. At this point, you can start up the Udacity provided vehicle simulator for the Path Planning project, and the path planner implementation will commence processing, driving the ego vehicle.

---
### Demo Video

A demo video of the path planner implementation driving the simulated ego vehicle on the highway for a total of appropriately 4.8 miles can be found in the /videos folder (path-planner-1.mp4). The path planner attempts to maintain a target speed of just under 50 mph. Further details on the implementation of the project can be found below.

---
### Implementation

#### 1. The car should drive valid trajectories, including:

* Driving at least 4.32 miles without incidents (incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes)
* Drives according to the speed limit (the car shouldn't drive faster than the speed limit; also the car shouldn't drive much slower than speed limit unless obstructed by traffic)
* Max Acceleration and Jerk are not exceeded (the car should not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3)
* The car does not have collisions (the car must not come into contact with any of the other cars on the road)
* The car stays in its lane, except for the time between changing lanes (the car doesn't spend more than 3 seconds outside of the lane lines while changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road)
* The car is able to change lanes (the car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic)

The following is how my model performs in general; I have made every attempt I could think of to have the ego vehicle do abide by all of the above rules for the project, but I cannot claim it is perfect. In the numerous tests I performed, it tended to operate properly approximately 95% of the time (or better). Occasionally though it exhibited some issues. These will be described further in the discussion below.

* Driving at least 4.32 miles without incident

My implementation can clearly do this adequately as demonstrated by the example video. I am confident that it will perform well upon actual testing, but I recognize that it isn't perfect.

* Drives according to the speed limit (of 50 mph)

I have designed the implementation to limit the speed of the ego vehicle to a maximum of 49.75 mph. This exact speed was chosen after empirical testing revealed that setting the speed to 50 mph exactly would cause an "over speed limit" incident to trigger, likely due to minor reporting glitches in the simulator. By limiting the speed to just below 50 mph, this is avoided.

In traffic, the ego vehicle will typically follow the flow of the traffic, maintaining a healthy distance from the lead vehicle (no tailgating!) to prevent collisions. If the traffic slows down in front of the ego vehicle, it will slow down as well, then ramp its speed up again after the situation passes.

* Max Acceleration and Jerk are not exceeded

I have not noticed either of these incidents occurring, given the nature and implementation of the path planning algorithm

* The car does not have collisions

The ego car in this implementation is not a daredevil. I struggled for hours trying to get the system to be more aggressive in lane changes and passing, and what I generally got for my troubles was a car that ran others off the road. So after lengthy experimentation and parameter tweaking, I settled on what you see in the project submission. The ego vehicle tends to remain in a lane once there, and only changes lanes when there is a very wide margin in terms of collision. This seems to work well, when it executes. However, lane changes aren't something the ego vehicle does frequently.

In my testing, I have found that when the rare collision occurs, it is due to one of two general scenarios, the first of which I have done my best to mitigate (which has resulted in the wide margin for choice):

* The ego vehicle changes lanes, but isn't moving fast enough as another vehicle moving much faster in the lane being changed to approaches. In the event the lane change isn't completed in time, a collision can occur if the other car doesn't brake in time
* More insidious is the fact that there are cars in the simulation that seem to act like "reckless drivers" at times. These cars will sometimes swerve in front of other cars or the ego vehicle, so close as to not allow the ego vehicle time to avoid a collision.

I have also seen these "reckless driver" vehicles cause collisions ahead of the ego vehicle, and I have seen vehicles from the opposite side veer into the right-hand lanes as well. It doesn't appear that when this occurs that these vehicles are in the sensor data - at least, my car and other cars don't appear to acknowledge they are anywhere in front of them. I don't know for certain if this is absolutely true, as these incidents are very rare occurrences, and are difficult to test for.

* The car stays in its lane, except for the time between changing lanes

For my submission, this is essentially true. There are cases (such as on certain turns in the road) where it appears that the wheels of the ego vehicle are just outside of the lane lines (or riding on top of them), but this doesn't seem to trigger the incident. In any case, the ego vehicle eventually moves back to the main "center" of the lane it is in.

* The car is able to change lanes

As described above, this does occur, but not with extreme frequency. In an effort to make the car safer, the code forces the behavior planner state machine to only perform a lane change while it is in a lane change preparation phase.

Without this provision in place, I was finding that the ego vehicle would sometimes change lanes without consulting the immediate states of the other vehicles, and as such collisions would happen fairly regularly. Forcing it to do a lane change prep before the actual lane change allows for the system to "stand down" if it seems anything out of the ordinary before it finally changes lanes. It's not 100% perfect, but it is mostly adequate for the job.

---
### Reflection on the Path Planner Process

The following is a brief rundown of each of the elements involved in the execution of the path planner and how the path to follow is generated. The explanations are presented in the same order as they were previously listed above in the code inclusion section at the beginning of this README file.

#### 1. Initial execution starts in main.cpp

Most of main.cpp is involved in incidental functioning, and wasn't altered much for the project.

The code first reads in the waypoint data from the ""/data/highway_map.csv" file, which essentially is a set of waypoint data information which follows the centerline of the simulated highway road.

The data consists of a series of data points, each as a global map x/y Cartesian coordinate pair, along with a Frenet coordinate pair of s (distance along the road) and d (distance from centerline). The "d" portion of this coordinate pair is curious in that it is given as an x/y unit vector pair "dx" and "dy". This vector pair is later converted into a unit vector which can be used to calculate distance from the centerline.

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

Notice that the only information we pass into the planner is the sensor fusion data and the ego vehicle's Frenet s-coordinate value. We need the sensor fusion data obviously, because it contains information about the state of the other vehicles on the road. But we only care about the ego vehicle's current s-value, because all the other values for the vehicle will be computed by the planner, but it is very important that we know the current s-value, for reasons that will be explained later (although the hint in the comment should be a big clue).

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

We then find the Frenet d-coordinates of the lane the ego vehicle is in, and then extrapolate a series of three x/y coordinates which follow the waypoints, but spaced 30 meters apart. These are also placed on the stack (PP.cpp, lines 140-151):

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

At this point, we have a series of five global x and y coordinates which describe a path from some given starting position to some ending position, which were given in Frenet s and d coordinates, then transformed to x/y Cartesian coordinates.

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

The vehicle handler class is used to represent individual vehicles on the road, including both the ego vehicle, and "non-player" simulated traffic vehicles. The representation includes location (in Frenet coordinates), velocity, acceleration, and "state", which is represents what the vehicle is currently doing or should be doing next:

* CS - "constant speed", which is only utilized at the instantiation of a vehicle
* KL - "keep lane", which means the vehicle needs to stay in its current lane
* PLCL & PLCR - "prepare for lane change left/right", which means the car needs to find a gap in the vehicles on its left or right, depending on which lane it intends to change lanes to
* LCL & LCR - "lane change left/right", which means the car needs to change lanes to the left or right

The simulated "non-player" vehicles are all configured as "constant speed", though this isn't really the case; because the system doesn't really care or know what any of these vehicles is doing, beyond what it can see based on their velocity and other information from the sensor data, the state designation acts only as a "placeholder" value.

The state instead is only used by the ego vehicle representation, to inform the system to update the trajectory as needed, and ultimately to alter the resulting path for the planner.

A vehicle is instantiated and configured via the constructor (vehicle.cpp, lines 16-37):

```c
Vehicle::Vehicle(int lane, double s, double v, double a, bool is_ego) {
    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = "CS";

    if (is_ego) {
      // configure speed limit, num_lanes, max_acceleration,
      // max_decceleration, and is_ego for ego vehicle
      this->target_speed = EGO_MAX_VELOCITY;
      this->lanes_available = NUM_LANES;
      this->max_acceleration = EGO_MAX_ACCEL;
      this->max_decceleration = EGO_MAX_DECEL;
      this->is_ego = true;
    } else {
      this->max_acceleration = 0;
      this->max_decceleration = 0;
    }

    this->cost = Cost();
}
```

When a vehicle is instantiated, information about the vehicle is passed in to the constructor regarding its current lane, s-value (Frenet coordinate), velocity, acceleration, and whether the ego vehicle is the one being instantiated or not. If the ego vehicle is being instantiated, then a variety of other class variables are configured using values from the configuration data (see config.h). In addition, the vehicle object is flagged as being the ego vehicle; this flag is necessary because other processes in the system need to know when the ego vehicle is being processed, and perhaps take alternative actions (for instance, earlier it was mentioned when the road handler class, road.cpp) populates traffic, it skips over the ego vehicle; while this is done using the id field of the map of vehicles, it is the same principle; we can't use the map id inside an individual vehicle because the object isn't aware of that setting, so the flag is there to indicate this difference).

After instantiation, the vehicle can then be manipulated as needed. The majority of methods within the class are for the exclusive use of the ego vehicle only; these include the following methods:

* Vehicle::update_state() - wrapper for getting the next state of the ego vehicle based on predictions of what the other vehicles are doing
* Vehicle::get_next_state() - gets the next state for the ego vehicle based on predictions of what the other vehicles are doing
* Vehicle::min_cost_state() - determines which state for the ego vehicle is best to transition to, based on the best (lowest) cost score
* Vehicle::trajectories_for_state() - simulates all states available from current state of ego vehicle, and develops trajectories for each
* Vehicle::realize_state() - based on the set ego vehicle state, updates the trajectory motion for the state
* Vehicle::realize_constant_speed() - keeps the ego vehicle at a constant speed by setting the acceleration to zero
* Vehicle::realize_keep_lane() - alters the acceleration only of the ego vehicle, based on what vehicles ahead are doing
* Vehicle::realize_lane_change() - alters the ego vehicle's current lane to either the left or right; also updates the acceleration for the new lane state
* Vehicle::max_accel_for_lane() - alters the ego vehicle's acceleration, and thus speed, based on whether cars are slowing down or speeding up ahead of the ego vehicle
* Vehicle::realize_prep_lane_change() - prepares the ego vehicle for a lane change, by looking for a suitable gap to the left or right of the ego vehicle, depending on which lane is being changed to
* Vehicle::generate_predictions() - generates a list of lane and s-values for a given vehicle over a set time horizon, to allow the ego vehicle a "prediction" of where a "non-player" car will be after the time period (or anywhere along the period), based on the known beginning state

The remainder of methods in the class are for the use of any instantiated vehicle; all methods and how their internal processes work will be described below.

---
After the constructor, the next method available in the vehicle handler class is an update method; this method allows a few internal state values of the vehicle instance to be updated (vehicle.cpp, lines 41-46):

```c
void Vehicle::update(int lane, double s, double v, double a) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
}
```

Following this method is the Vehicle::update_state() method, which as noted above, is merely a wrapper which calls the get_next_state() method, and updates the current state of the ego vehicle based on predictions made earlier in the road handler class (vehicle.cpp, line 83):

```
this->state = this->get_next_state(predictions);
```

After Vehicle::get_next_state() is passed these predictions, it begins a process to determine which states can be reached from the ego vehicle's currently defined state, based on what state it is in, and what lane it is in (vehicle.cpp, lines 87-114):

```c
vector<string> states = {"KL", "PLCR", "LCR", "PLCL", "LCL"};

// remove states that are impossible to get to from the current state
string state = this->state;

if (this->lane == 0) {  // left-hand lane
  states.erase(states.begin() + 4);
  states.erase(states.begin() + 3);

  // if in KL state then only allow
  // transition to PLCR, not LCR
  if (state.compare("KL") == 0) states.erase(states.begin() + 2);

} else if (this->lane == 1) {  // middle lane
  // if in KL state, only allow transition
  //  to PLCR or PLCL, not LCR or LCL
  if (state.compare("KL") == 0) {
    states.erase(states.begin() + 4);
    states.erase(states.begin() + 2);
  }
} else if (this->lane == this->lanes_available - 1) {  // right-hand lane
  // if in KL state, only allow transition
  // to PLCR, not LCR
  if (state.compare("KL") == 0) states.erase(states.begin() + 4);

  states.erase(states.begin() + 2);
  states.erase(states.begin() + 1);
}
```

For instance, if it is in the far left lane, there isn't a reason to allow it to go any further left, so those states are removed from the list of states it can potentially transition to. Furthermore, if the ego vehicle is currently in a "keep lane" state, it isn't allowed to transition directly to a "lane change" state, but must first be in a "prepare lane change" state instead (this is a safety measure, because in the "lane change" state, no checking is done to see if the lane being changed to is clear of vehicles in order to safely change lanes; the "prepare lane change" state however does check for this).

Once it has a list of states which can be used, for each one of those states it prepares a set of trajectories and calculates the cost for each one of those trajectories, and builds a list of the potential new states and their associated costs (vehicle.cpp, lines 119-129):

```c
for (int i = 0; i < states.size(); i++) {
  string state = states[i];

  // two trajectories - the current trajectory, and the proposed state trajectory
  vector<Snapshot> trajectories = this->trajectories_for_state(state, predictions, TRAJECTORIES_HZ);

  double cost = this->cost.calculate_cost(*this, trajectories, predictions);

  new_states.insert(new_states.end(), state);
  new_costs.insert(new_costs.end(), cost);
}
```

Then using the lists, it determines which cost is best (lowest), and returns that as the best state to be in (vehicle.cpp, lines 131-133):

```c
string best = this->min_cost_state(new_states, new_costs);

return best;
```

Best cost is determined by the next method, Vehicle::min_cost_state(), which essentially loops thru the costs and states, and finds whichever one in the list has the lowest cost, and returns the associated state for that lowest cost (vehicle.cpp, lines 142-152):

```c
for (int i = 0; i < states.size(); i++) {
  if (DEBUG) {
    if (i > 0) cout << ", ";
    cout << states[i] << ": " << costs[i];
  }

  if (costs[i] < best_cost) {
    best_cost = costs[i];
    best_state = states[i];
  }
}
```

The trajectories for each potential future state the ego vehicle can be in is determined by the next method, Vehicle::trajectories_for_state(). This method takes the potential state, the predictions of what the other vehicles on the road are doing, and a horizon time, and uses those values to project out where the ego vehicle will end up at, based on it's current state, if that proposed state is utilized. It does this by simply simulating what the vehicle will do over the time horizon, given the initial state of the vehicle (vehicle.cpp, lines 160-181):

```c
// remember the current state of ego vehicle
Snapshot current = Snapshot(this->lane, this->s, this->v, this->a, this->state);

// build a list of trajectories
vector<Snapshot> trajectories;

// save the current state for the initial trajectory in the list
trajectories.insert(trajectories.end(), current);

// ...pretend to be in the new proposed state
this->state = state;

// perform the state transition for the proposed state
this->realize_state(predictions);

for (int i = 0; i < horizon; i++) {
  // update the velocity and acceleration of ego out to the horizon
  this->increment(1.0, true);
}

// save the trajectory results of the proposed state
trajectories.insert(trajectories.end(), Snapshot(this->lane, this->s, this->v, this->a, this->state));
```

Once this is done, the original state is restored, and the trajectories (which consist of the current trajectory, and a new potential trajectory for that potential state) are returned to be evaluated for cost (see Vehicle::get_next_state() discussed earlier). How the cost(s) are computed is the subject of a later section, and won't be elaborated here (see below in section 7 - Execution of the cost calculations class).

We now come upon a variety of "housekeeping" methods used both within the class and outside of it. First is the Vehicle::display() method (vehicle.cpp, lines 207-216):

```c
string Vehicle::display() {
  ostringstream oss;

  oss << "s:    " << this->s << "\n";
  oss << "lane: " << this->lane << "\n";
  oss << "v:    " << this->v << "\n";
  oss << "a:    " << this->a << "\n";

  return oss.str();
}
```

Its purpose is mainly that of a debugging aid which, when called, returns a string of information which can be logged or printed out to the console. It was provided in a portion of the "Implement a behavior planner class in C++" quiz lesson. I kept it around in the code, but I don't believe I have utilized it anywhere.

Next is the Vehicle::increment() method. This method calculates the next s-value and velocity of the vehicle for a given time delta (dt), based on its current velocity and acceleration (vehicle.cpp, lines 222-235):

```c
void Vehicle::increment(double dt, bool overide) {
  double ddt = dt * SECS_PER_TICK;

  // NOTE: if we are updating the ego car, don't calculate the
  // s-value, keep the currently set value and don't vary it
  if (!this->is_ego || overide) this->s += this->v * ddt;

  this->v += this->a * ddt;

  if (this->is_ego) {
    this->v = max(0.0, this->v);  // don't allow ego velocity to go negative
    this->v = min(EGO_MAX_VELOCITY, this->v);  // ...or go over the speed limit either
  }
}
```

If the vehicle being updated is the ego vehicle, the update of the s-value is skipped (because we want to only use the simulator's value for the ego vehicle as canonical), and the velocity is kept within a range between 0 mph and the maximum velocity for the ego (which is set to 49.75 mph for our purposes, in config.h).

Next is the Vehicle::state_at() method. This method predicts the state of a vehicle after delta time (t), assuming constant acceleration (vehicle.cpp, lines 237-246):

```c
vector<double> Vehicle::state_at(double t) {
  //
  // Predicts state of vehicle in t*0.02 seconds (assuming constant acceleration)
  //
  double dt = t * SECS_PER_TICK;
  double s = this->s + this->v * dt + this->a * dt * dt / 2;
  double v = this->v + this->a * dt;
  if (this->is_ego) v = max(0.0, v);  // don't allow the ego's velocity to go negative
  return {static_cast<double>(this->lane), s, v, this->a};
}
```

In addition, if the ego vehicle is the one being checked, it's predicted velocity is not allowed to go negative.

The next method, Vehicle::collides_with(), is used to perform a simple collision detection check between the current vehicle (this) and another vehicle passed to the method (vehicle.cpp, lines 248-256):

```c
bool Vehicle::collides_with(Vehicle other, double at_time) {
  //
  // Simple collision detection.
  //
  vector<double> check1 = this->state_at(at_time);
  vector<double> check2 = other.state_at(at_time);

  return (check1[0] == check2[0]) && (abs(check1[1] - check2[1]) <= 1);
}
```

The routine basically gets the predicted state of both vehicles, then uses the results to check to see if the vehicles are in the same lane and their s-values are close to each other or equal. Rough, but it gets the job done. Based on those checks, a true or false value is returned to the caller, which is the next method we encounter.

The Vehicle::will_collide_with() method uses the previous two methods to evaluate a set of time steps, and determine if and when a collision between the current vehicle and another vehicle can occur (vehicle.cpp, lines 258-272):

```c
Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {
  Vehicle::collider collider_temp;
  collider_temp.collision = false;
  collider_temp.time = -1;

  for (int t = 0; t < timesteps + 1; t++) {
    if (this->collides_with(other, static_cast<double>(t))) {
      collider_temp.collision = true;
      collider_temp.time = t;
      return collider_temp;
    }
  }

  return collider_temp;
}
```

As can be seen, it does this using an iterative approach, and if it finds that a collision occurs, it updates a structure it returns with that information.

Next come the Vehicle::realize_state() method. It is basically a wrapper method, which takes the previously generated predictions (see road.cpp), and based on the current state of the vehicle it calls one of several different realizer methods, which ultimately carry out the update of the ego vehicle's state (vehicle.cpp, lines 274-294):

```c
void Vehicle::realize_state(map<int, vector<vector<double>>> predictions) {
  //
  // Given a state, realize it by adjusting acceleration and lane.
  // Note - lane changes happen instantaneously.
  //
  string state = this->state;

  if (state.compare("CS") == 0) {
     realize_constant_speed();
  } else if (state.compare("KL") == 0) {
     realize_keep_lane(predictions);
  } else if (state.compare("LCL") == 0) {
     realize_lane_change(predictions, "L");
  } else if (state.compare("LCR") == 0) {
     realize_lane_change(predictions, "R");
  } else if (state.compare("PLCL") == 0) {
     realize_prep_lane_change(predictions, "L");
  } else if (state.compare("PLCR") == 0) {
     realize_prep_lane_change(predictions, "R");
  }
}
```

As noted in the comments, a "lane change" happens "instantly" - at least from the viewpoint of the vehicle class; however, the update of the path in the path planner (PP.cpp) causes the actual lane change to happen gradually and smoothly, and avoids any uncomfortable jerk. While not an accurate representation in reality, in practice this approximation approach appears to work ok.

Next are each of the realizer methods, the first of which is Vehicle::realize_constant_speed(), which essentially sets the vehicle's current acceleration to 0.

Next is the realizer Vehicle::realize_keep_lane(), which only changes the acceleration for the vehicle by calling another method that determines the maximum acceleration possible for the vehicle's current lane (vehicle.cpp, lines 301-304):

```c
void Vehicle::realize_keep_lane(map<int, vector<vector<double>>> predictions) {
  // continue to get the acceleration for the current lane
  this->a = this->_max_accel_for_lane(predictions, this->lane, this->s);
}
```

The next realizer is for lane changing, Vehicle::realize_lane_change(). It is passed a directional flag ("L" or "R") for the direction the ego vehicle should change lanes to, which is then used to determine a delta offset from the current lane, which is then updated. A helper function is called to prevent the lane value from going below zero or over the maximum number of lanes, and the acceleration is matched for the new lane (vehicle.cpp, lines 306-324):

```c
void Vehicle::realize_lane_change(map<int, vector<vector<double>>> predictions, string direction) {
  // doing a lane change, so based on which direction (left/right)
  // for the new lane being changed to, set a delta offset to
  // represent the new lane
  double delta = 1;

  if (direction.compare("L") == 0) {
    delta = -1;
  }

  // change to the new lane
  this->lane += delta;

  // prevent straying beyond minimum or maximum lane positions
  this->lane = minmaxCarLaneNumber(this->lane);

  // get acceleration for new lane
  this->a = this->_max_accel_for_lane(predictions, this->lane, this->s);
}
```

The next method is the one that determines that maximum acceleration for the lane, Vehicle::max_accel_for_lane() - I should note that I have, at times here in this writeup, altered the name of this method slightly to remove the underscore that precedes the method name; this has been done due to how my editor highlights things; if it begins with an underscore, it changes color of all text afterward to purple, until another underscore is encountered. Thus, utilizing the underscore in the name causes major visual issues in my editor, so I have opted to, at times, to not reference the underscore. I should probably just update the code, but at this late junction, I am leaving things as-is for now.

This method operates by first calculating an acceleration for the vehicle based on what the current velocity is and what the target velocity needs to be at each time step this method is called. Eventually, the delta will converge to zero (vehicle.cpp, lines 327-330):

```c
// at each time step we need to gradually approach the target
// speed by ramping up the acceleration, to its maximum limit
double delta_v_til_target = this->target_speed - this->v;
double max_acc = min(this->max_acceleration, delta_v_til_target);
```

After this, we then use the predictions to find what vehicle are ahead of the ego vehicle in its lane (vehicle.cpp, lines 336-346):

```c
while (it != predictions.end()) {
  int vv_id = it->first;

  vector<vector<double>> vv = it->second;

  if ((static_cast<int>(vv[0][0]) == lane) && (vv[0][1] > ss)) {
    in_front.push_back(vv);
  }

  it++;
}
```

If there are any found ahead of the ego vehicle, the one closest to the ego vehicle (directly ahead) is determined, and then based on that, the acceleration of the ego vehicle is altered depending on how close the ego vehicle is to that leading vehicle, and based on the configured "buffer zone" as well (vehicle.cpp, lines 348-384):

```c
// if there are any cars in front of the ego car, find
// the first car in the lane (the one directly ahead)
this->in_front = in_front.size();

if (this->in_front > 0) {
  double min_s = LEADING_HZ;  // how far to look ahead

  vector<vector<double>> leading;

  for (int i = 0; i < in_front.size(); i++) {
    if ((in_front[i][0][1] - ss) < min_s) {
      min_s = (in_front[i][0][1] - ss);
      leading = in_front[i];
    }
  }

  if (leading.size() > 1) {
    // now that we have found the car directly in front of the
    // ego car, find where it will be next...
    double next_pos = leading[1][1];
    // then find where the ego car will be next based on its speed
    double my_next = ss + this->v;
    // find out how far apart they will be from each other at that time
    double separation_next = next_pos - my_next;
    // subtract a bit of buffer room for comfort, and that's the
    // available room the ego car has to maneuver in
    double available_room = separation_next - PREFERRED_BUFFER;

    // keep going at current speed if there is available room,
    // otherwise reduce speed...
    max_acc = min(max_acc, available_room);
    // but don't let the acceleration fall below the minimum
    max_acc = max(max_acc, -this->max_decceleration);
  }
}

return max_acc;
```

The acceleration value is then returned.

The next realizer is for preparing a lane change. Vehicle::realize_prep_lane_change() does this in a manner somewhat similar to what is going on in previous methods, with some differences. The two major differences is that it doesn't change the lanes, but it does check the lane that is going to be changed to by the ego vehicle. It also checks to the immediate side and behind the ego vehicle, and not ahead of it.

Like the  Vehicle::realize_lane_change() method, it is passed a directional flag ("L" or "R") for the direction the ego vehicle should change lanes to, which is then used to determine a delta offset from the current lane. Then it determines whether there are any vehicles in that lane next to the ego vehicle, or behind it (vehicle.cpp, lines 387-419):

```c
void Vehicle::realize_prep_lane_change(map<int, vector<vector<double>>> predictions, string direction) {
  // prepping for a lane change, so based on which lane is
  // going to be changed to, set a delta offset to represent
  // the new lane
  int delta = 1;

  if (direction.compare("L") == 0) {
    delta = -1;
  }

  // set the lane to check based on the delta offset value
  int lane = this->lane + delta;

  // build a list of all vehicles next to or behind the ego
  // car in that lane
  map<int, vector<vector<double>>>::iterator it = predictions.begin();

  vector<vector<vector<double>>> at_behind;

  while (it != predictions.end()) {
    int v_id = it->first;

    vector<vector<double>> v = it->second;

    if ((static_cast<int>(v[0][0]) == lane) && (v[0][1] <= this->s)) {
      at_behind.push_back(v);
    }

    it++;
  }

  // if there are any cars found behind the ego car...
  this->at_behind = at_behind.size();
  ...
}
```

If any other vehicles are found, the closest one to the ego vehicle is located, and calculations are done to alter the ego vehicle's acceleration to make it get behind the trailing vehicle, and hopefully near a "slot" between it and any other vehicle behind it (vehicle.cpp, lines 421-474):

```c
if (this->at_behind > 0) {
  double max_s = -FOLLOW_HZ;  // how far to look behind

  // find the closest vehicle behind the ego vehicle
  vector<vector<double>> nearest_behind;

  for (int i = 0; i < at_behind.size(); i++) {
    if ((at_behind[i][0][1]) > max_s) {
      max_s = at_behind[i][0][1];
      nearest_behind = at_behind[i];
    }
  }

  // get the diffence in the velocity of the trailing vehicle
  double target_vel = nearest_behind[1][1] - nearest_behind[0][1];
  // the ego car needs to change from its current velocity
  double delta_v = this->v - target_vel;
  // ...and the ego car needs to get behind the trailing vehicle
  double delta_s = this->s - nearest_behind[0][1];

  // if the trailing car is changing its velocity...
  if (delta_v != 0) {
    // ...then we need to compute the acceleration value needed
    // to match the change in order to get behind it
    double time = -2.0 * delta_s / delta_v;
    double aa;

    if (time == 0) {
      aa = this->a;
    } else {
      aa = delta_v / time;
    }

    // keep acceleration within boundaries of min/max acceleration
    if (aa > this->max_acceleration) {
      aa = this->max_acceleration;
    }

    if (aa < -this->max_decceleration) {
      aa = -this->max_decceleration;
    }

    this->a = aa;
  } else {
    // if the trailing vehicle isn't changing its speed, then the
    // ego car just needs to slow down
    double my_min_acc = max(-this->max_decceleration, -delta_s);

    this->a = my_min_acc;
  }
} else {
  // otherwise with no cars found behind ego, continue at regular speed
  this->a = this->_max_accel_for_lane(predictions, this->lane, this->s);
}
```

Otherwise, if no trailing vehicles are found, then the acceleration for the lane is updated as before.

Finally, the last method of the vehicle class is the vehicle prediction generation method, Vehicle::generate_predictions(). This method is passed a "time horizon", which is then used to iteratively call the Vehicle::state_at() method to determine a set of predictions of where the vehicle will be, over time, given its current velocity and assuming a constant acceleration. As noted before, these prediction are made in road.cpp for all of the non-ego vehicles on the road, which are then passed into the ego vehicle to determine what its new state should be, given these predictions and its current state, plus the costs associated with changing to a new state vs continuing with the current state.

#### 7. Execution of the cost calculations class (cost.cpp)

The purpose of the cost calculations class is to take a potential new trajectory for the ego vehicle, plus the predictions for the states of all the other vehicles, and from the vantage point of the ego vehicle, calculate a cost for that trajectory, based on one or more cost functions. In the case of this implementation, there are currently three cost functions defined:

* Cost::inefficiency_cost() - determines the cost of the trajectory from the standpoint of how efficient it is at keeping the ego vehicle near the target speed (ie, fastest travel time)
* Cost::collison_cost() - determines the cost of the trajectory based on how likely it is to cause a collision
* Cost::buffer_cost() - determines the cost of the trajectory based on how close or far away it will take us to a nearby vehicle

These costs are all added up, and this determines the total overall cost returned back to the ego vehicle, which is then used to select the trajectory with the lowest cost, as being the trajectory to best go with for the next time step.

The main entry point to this process is Cost::calculate_cost(). This method first calls an internal method to get some data about the trajectory, which it can then pass on to the cost functions themselves for their use as needed. Once the cost is computed, it is returned (cost.cpp, lines 14-24):

```c
double Cost::calculate_cost(const Vehicle &vehicle, vector<Snapshot> trajectories, map<int, vector<vector<double>>> predictions) {
  TrajectoryData trajectory_data = this->get_helper_data(vehicle, trajectories, predictions);

  double cost = 0;

  cost += this->inefficiency_cost(vehicle, trajectories, predictions, trajectory_data);
  cost += this->collision_cost(vehicle, trajectories, predictions, trajectory_data);
  cost += this->buffer_cost(vehicle, trajectories, predictions, trajectory_data);

  return cost;
}
```

The helper method called, which is a part of the class, is Cost::get_helper_data(). It basically does computations for average speed, closest approach of vehicles in other lanes (based on their predictions), as well as other useful data which can be used by the cost functions (cost.cpp, lines 31-80):

```c
Snapshot current_snapshot = t[0];
Snapshot first = t[1];
Snapshot last = t[t.size() - 1];

double dt = trajectories.size();

trajectory_data.proposed_lane = first.lane;

trajectory_data.avg_speed = (last.s - current_snapshot.s) / dt;

// initialize a bunch of variables
vector<double> accels;

double closest_approach = 9999999.0;
bool collides = false;

map<int, vector<vector<double>>> filtered = this->filter_predictions_by_lane(predictions, trajectory_data.proposed_lane);

// get trajectory for future state of ego vehicle
Snapshot snapshot = trajectories[1];

accels.insert(accels.end(), snapshot.a);

map<int, vector<vector<double>>>::iterator it = filtered.begin();

while (it != filtered.end()) {
  int v_id = it->first;

  vector<vector<double>> v = it->second;
  vector<double> curr_state = v[1];
  vector<double> prev_state = v[0];

  bool vehicle_collides = this->check_collision(snapshot, prev_state[1], curr_state[1]);
  if (vehicle_collides) {
    trajectory_data.collides = true;
    trajectory_data.collides_at = curr_state[1];
  }

  int dist = abs(curr_state[1] - snapshot.s);
  if (dist < trajectory_data.closest_approach) trajectory_data.closest_approach = dist;

  it++;
}

// find max acceleration value
int num_accels = accels.size();
trajectory_data.max_acceleration = 0;
for (int i = 0; i < num_accels; i++) {
  if (accels[i] > trajectory_data.max_acceleration) trajectory_data.max_acceleration = accels[i];
}
```

As a part of that process, a few other internal functions are called, Cost::filter_predictions_by_lane() and Cost::check_collision(), which are both essentially straightforward in their operation (cost.cpp, lines 85-112):

```c
map<int, vector<vector<double>>> Cost::filter_predictions_by_lane(map<int, vector <vector<double>>> predictions, int lane) {
  map<int, vector<vector<double>>> filtered;

  map<int, vector<vector<double>>>::iterator it = predictions.begin();

  while (it != predictions.end()) {
    int v_id = it->first;
    vector<vector<double>> predicted_traj = it->second;

    if (static_cast<int>(predicted_traj[0][0]) == lane && v_id != -1) {
      filtered[v_id] = predicted_traj;
    }

    it++;
  }

  return filtered;
}

bool Cost::check_collision(Snapshot snapshot, double s_prev, double s_curr) {
  double v_target = s_curr - s_prev;

  if (s_prev < snapshot.s) return (s_curr >= snapshot.s);
  if (s_prev > snapshot.s) return (s_curr <= snapshot.s);
  if (s_prev == snapshot.s) return (v_target <= snapshot.v);

  return true;
}
```

Next are the cost functions themselves. The first, Cost::inefficiency_cost(), is used to determine how efficient (or close to the target speed) the average speed of the vehicle is. It does this by taking the average speed and getting the delta between it and the target speed. This value is then divided by the target speed to yield a "percentage of target speed", which is squared, and that value is used as a multiplier with the EFFICIENCY cost constant defined in the configuration header (config.h). It is easy to see how this works: As long as the delta is some number greater than zero, there will always be an output cost of something greater than zero, but as it approaches zero, that multiplier will become smaller and smaller, ultimately being so close to zero (extremely tiny) so as to make the resulting cost very small. An inverse relationship - the closer to the target speed, the lower the cost reported (cost.cpp, lines 114-121):

```c
double Cost::inefficiency_cost(Vehicle vehicle, vector<Snapshot> trajectories, map<int, vector<vector<double>>> predictions, TrajectoryData data) {
  double speed = data.avg_speed;
  double target_speed = vehicle.target_speed;
  double diff = target_speed - speed;
  double pct = diff / target_speed;
  double multiplier = pct * pct;
  return multiplier * EFFICIENCY;
}
```

The second cost function, Cost::collision_cost(), determines the cost of the trajectory based on whether it will cause a collision with another vehicle, and when. If there is no collision possible, based on the trajectory data calculated prior, a cost of zero is reported back (best case scenario). Otherwise, if a collision is determined to occur, the time until the collision is noted, and this is squared. This is treated as a negative exponent which is used to calculate the exponential of, the result which serves as the multiplier to the COLLISION cost constant. It can be seen that if the time to collision is some number greater than zero, then the multiplier gets smaller (and approached zero) as that time gets larger. As that time gets smaller (approaching zero) that multiplier gets larger (and if it is equal to zero - the multiplier equals 1). So again, an inverse relationship is established where as long as the time to the accident is large, the cost is low (cost.cpp, lines 123-132):

```c
double Cost::collision_cost(Vehicle vehicle, vector<Snapshot> trajectories, map<int, vector<vector<double>>> predictions, TrajectoryData data) {
  if (data.collides) {
    double time_til_collision = data.collides_at;
    double exponent = time_til_collision * time_til_collision;
    double multiplier = exp(-exponent);
    return multiplier * COLLISION;
  }

  return 0.0;
}
```

The third and final cost function, Cost::buffer_cost(), determines the cost of the trajectory based on how close the ego vehicle can potentially get to another vehicle for that trajectory. The closer this is, the greater the danger to the vehicles. If the closest approach is zero (ie, collision), the cost is reported as ten times the value of the DANGER cost constant defined in the configuration header (config.h). Otherwise, a calculation is done to see how far away, based on the average speed of the ego vehicle, the ego vehicle will be at closest approach. If the distance is greater than a defined buffer preference, zero cost is returned. Otherwise a multiplier is determined by taking the distance found, dividing it by the buffer preference, and squaring it, then subtracting that value from one. Again here, it can be seen that as the distance decreases and becomes close to zero the multiplier becomes equal to 1 (maximum danger). As the distance becomes larger, the multiplier can grow closer to zero, or even go negative, with an associated difference in cost (cost.cpp, lines 134-144):

```c
double Cost::buffer_cost(Vehicle vehicle, vector<Snapshot> trajectories, map<int, vector<vector<double>>> predictions, TrajectoryData data) {
  double closest = data.closest_approach;
  if (closest == 0) return 10 * DANGER;

  double timesteps_away = closest / data.avg_speed;
  if (timesteps_away > PREFERRED_BUFFER) return 0;

  double multiplier = 1 - pow((timesteps_away / PREFERRED_BUFFER), 2);

  return multiplier * DANGER;
}
```

Again, it should be noted that in all of these cases, the cost functions all display an inverse relationship, so that something that was dangerous or was wanting to be avoided is given a larger cost versus that which is wanted in the end result. Further cost functions could be determined and utilized to come to a better decision for the trajectory as needed, but such cost functions should follow a similar pattern.

#### 8. Description of various helper functions (helper.cpp)

The helper functions (helper.cpp) are used by various other processes in the system to perform various general operations as needed. Since these functions may be used in multiple places, they are collected as a set and included in each area which needs access to them. They aren't defined as a class, though they could be. Instead, they are more generally scoped to make use of them easier.

The first two functions defined are used to convert degrees to radians, and radians to degrees. This is useful where yaw may be reported in different forms and needs to be converted for use in various other trigonometric methods (helper.cpp, lines 7-9):

```c
// For converting back and forth between radians and degrees.
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
```

Next is a simple distance calculation function (sum the squares of differences, take the square root) - see lines 11-13:

```c
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
```

The next two methods, ClosestWaypoint() and NextWaypoint(), deal with finding waypoints given the waypoint data read in main.cpp. The difference between the two functions is subtle, yet important (helper.cpp, lines 15-48):

```c
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y) {
  double closestLen = 100000;  // large number

  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];

    double dist = distance(x, y, map_x, map_y);

    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y) {
  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = abs(theta-heading);

  if (angle > pi() / 4) closestWaypoint++;

  return closestWaypoint;
}
```

The function ClosestWaypoint() basically takes the data, and an x/y Cartesian coordinate, and uses that to find the waypoint closest to that x/y position. This is fine and good for general waypoint finding, but in the case of a vehicle, you will probably instead want a waypoint not closest to the vehicle, but close to and ahead of the vehicle based on the vehicle's yaw angle. So, if the car is driving from waypoint to waypoint, you want to be able to say "Given my current position at or near this waypoint, and my current heading, what is the next waypoint?"

If ClosestWaypoint() were used, the waypoint returned would be the one the vehicle was near at that moment; that is, the one it is currently at. Instead, you want to find the next waypoint in the direction the vehicle is facing. That is what NextWaypoint() does, by first finding the closest waypoint near the vehicle, and then using the angular difference between the vehicle's position and that waypoint, and the vehicle's current heading, in order to find the next waypoint.

Of course, depending on how the car is facing, the "next waypoint" could very well be the current waypoint...

The next two functions are used to convert between x/y Cartesian coordinates and Frenet s and d-value coordinates, given the waypoint data. I won't pretend to understand exactly what is going on in these functions, except to say that the getFrenet() function gets the s and d Frenet coordinates given the x/y Cartesian coordinates and the map waypoint data (helper.cpp, lines 50-90):

```c
// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y) {
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;

  if (next_wp == 0) {
    prev_wp  = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  // see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) frenet_d = -frenet_d;

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}
```

...and the getXY() function gets the x and y Cartesian coordinates given the s/d Frenet coordinates and the map waypoint data (helper.cpp, lines 92-116):

```c
// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < static_cast<int>(maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));

  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}
```

The next helper function, getLaneNumber(), returns a lane number (0, 1, or 2) given the Frenet d value (helper.cpp, lines 118-125):

```c
// Returns the lane # (0, 1, 2) given a frenet D value
int getLaneNumber(double d) {
  if (d >= 0 && d < LANE_WIDTH) return 0;
  if (d >= LANE_WIDTH && d < LANE_WIDTH * 2) return 1;
  if (d >= LANE_WIDTH * 2 && d < LANE_WIDTH * 3) return 2;

  return -1;  // out of lane boundaries
}
```

The next helper function, getLaneFrenet(), does the opposite - given a lane number, it returns the Frenet d-value for the center of the lane (helper.cpp, lines 127-130):

```c
// Returns the center of lane D value for a lane
int getLaneFrenet(int lane, int offset) {
  return 2 + 4 * lane + offset;
}
```

Lastly, the final helper function, minmaxCarLaneNumber(), restricts a lane number passed to it to between 0 and 2 (helper.cpp, lines 132-134):

```c
int minmaxCarLaneNumber(int lane) {
  return max(0, min(2, lane));
}
```

#### 9. Snapshot handler class (snapshot.cpp)

The snapshot handler class is very basic in how it works; essentially it acts as a structure to take a "snapshot" of the state of a vehicle, storing the lane, Frenet s-value, velocity, acceleration, and state within the structure. Declaring it as a class also allowed the use of it within vectors as a type, thus allowing the storage of multiple values with different types within each element of the vector - where such a use was needed (snapshot.cpp, line 7-13):

```c
Snapshot::Snapshot(int lane, double s, double v, double a, string state) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  this->state = state;
}
```

Since the variable of the resulting object are public, reading them back poses no challenge, nor did it need any special additional methods defined. I could have defined a "getter" method, and made the class variables private, but this would introduce unnecessary complexity to the class for little additional functionality.

---
### Simulation

#### 1. The vehicle must successfully drive a lap around the track

Upon running the simulation, the vehicle should navigate the highway course successfully and smoothly for a total of 4.35 miles, at speeds up to 50 mph without any incidents, similar to the example video.

#### 2. Improvements for exploration

There are many future improvements which could be made to the code, several of which are fairly obvious, and others which may or may not be necessary:

1. Merging PP::GenerateKeepLane() into PP::GeneratePath() would be a worthy improvement, as there isn't a need for these methods to be separate, and indeed the names can cause confusion. Initially I had thought I would need to create separate path planners for "keep lane" vs "change lane left" vs "change lane right", but the video walk-through code showed that to be a false notion. But the names I used stuck, and I never got around to cleaning up the code to reflect that the path generation was now a generalized method.

2. A refactor of certain areas of the code would be ideal; for instance, making the behavior planner class more functional, and less like a wrapper, by refactoring and reorganizing road.cpp and vehicle.cpp, and somehow merging them into BP.cpp. It may be that "road.cpp" should be put into BP.cpp, and the Vehicle class left alone. I'm not sure what the proper solution is there.

3. Refactor out dead or unused code. I know there's a bit of it in there...

4. I believe that in certain areas I may have misunderstood the intent or need for certain code; for instance, in the Vehicle class there's code involved that doesn't seem like it means anything proper in the context in which it is used. Similarly, there's code (and lack of use of data, like predictions in many of the cost functions) that don't seem properly used either. In general, a more in-depth look at all this, and a refactor of function and methods needs to be done to make this make better sense.

5. I need to find out why my vehicle acts "timid" in acceleration and deceleration. I tried to figure this out; I believe it's some kind of interplay between the cost functions for collision and/or buffer, and in the acceleration routines. I also believe the "prep lane change" function has some blame for this as well. The complex interplay and dependency of all these functions make finding causes difficult and time consuming.

6. I believe there needs to be some kind of a check in the "realize lane change" method in the Vehicle class to double-check the lane being changed to just before it does so, and if anybody is nearby, don't do the lane change. In theory, though, this is what the "prepare lane change" method is for, but it doesn't seem to be working properly all the time, I'm not sure.

7. I think that maybe an extra cost function or two needs to be put into place. For instance, something addressing the prepare lane change state, costs for that, might be useful.
