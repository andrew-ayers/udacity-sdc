[//]: # (Image References)

[image1]: ./images/self_driving_car_nanodegree_program.png "Title Image"

# **<center>Term 3 - Project 1: Path Planning - Part 1: Goals</center>**

<center>![alt text][image1]</center>

## Goals

* Design a path planner that is able to create smooth, safe paths for the car
* The car should follow along a 3 lane highway with traffic inside its lane
* The car has to avoid hitting other cars, and pass slower moving traffic
* The car can perform this using localization, sensor fusion, and map data

---
## Specification  ([rubric](https://review.udacity.com/#!/rubrics/1020/view))

---

### Compilation

#### 1. Submission code is included

It should be noted that the original bare project as received from Udacity consisted of only a single file, main.cpp. Due to the complex nature of the project, and the number of interrelated yet mostly independent sub-processes involved, I decided to structure the project as a series of classes to enable better focus and understanding of each part. The submission therefore includes the following files in the /src folder which form the implementation of the Path Planner:

* main.cpp - contains the base code to instantiate and run the path planner
* config.h - header file included by other files for various constant parameter definitions
* PP.h & PP.cpp - path planner class: handles dispatch to the remainder of process, and eventual path generation
* BP.h & BP.cpp - behavior planner class: instantiator and wrapper for Road class

The following files are my attempt at converting the python example code provided in Lesson 4: Behavior Planning; it was found by another user (see https://discussions.udacity.com/t/quiz-4-16-puzzling-python-official-solution-does-not-work/335092) that the python code does not actually work for that lesson, and that the car just follows along in lane one throughout the entire run (and never reaches the goal).

After implementing this, I found the same thing happening within the context of this project, and found the user's posting to the discussion forum as a result. Ultimately, in the effort to fix this issue and others, the code in areas no longer strictly follows the original python (not to mention that the code for that quiz was more step-like than continuous, which also necessitated various changes on my part to get it to work for the path planner):

* road.h & road.cpp - road handler class: handles populating the "road" for the behavior planner, and timestep advancing the behavior planner simulation
* vehicle.h & vehicle.cpp - vehicle handler class: handles a variety of tasks all relating to vehicles on the simulated road, including both the ego vehicle (our vehicle the simulation is driving) and the "np" vehicles (all the other "non-player" simulated traffic vehicles around the ego vehicle)
* cost.h & cost.cpp - cost calculations class: handles the cost calculations for the behavior planner

Finally, the following files are support library code used by the above processes:

* helper.h & helper.cpp - various helper functions for finding waypoints, converting between frenet and cartesian road coordinates, and others (most functionality provided by Udacity, with a few additional custom functions added)
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

Continue reading with Part 2: Reflection
