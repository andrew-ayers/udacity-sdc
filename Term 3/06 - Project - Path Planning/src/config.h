#ifndef CONFIG_H
#define CONFIG_H

using namespace std;

// general settings (DO NOT TOUCH)
const bool DEBUG = true;

const double MPH_TO_MPS = 2.24;     // MPH to meters-per-second, 2.24mph -> 1 m/s
const double SPLINE_SPACING = 30;   // 30 meters between spline segments
const int MAX_TICKS = 20;           // number of simulator "ticks" before behavior planner processing for ego
const double SECS_PER_TICK = 0.02;  // number of elapsed seconds per "tick" of simulator
const int FILLER_DIST = 50;         // 50 meters

// ego vehicle settings
const int EGO_START_LANE = 1;       // lane the ego vehicle starts in (fixed in simulator)
const double EGO_MAX_ACCEL = 11.0;  // +/- 11 meters-per-second; 11 * .02 = .22, close to the 0.224 value from walkthru
const double EGO_MAX_VELOCITY = 49.75;

// road settings
const int NUM_LANES = 3;

// horizon settings
const int PREFERRED_BUFFER = 25;  // 0.5s buffer (25 * 0.02) - impacts "keep lane" behavior.

const int PREDICTIONS_HZ = 125;   // 2.5 second horizon (125 * 0.02)
const int TRAJECTORIES_HZ = 125;

const double LEADING_HZ = 250.0;  // how far out directly ahead of us to find the closest car 500
const double FOLLOW_HZ = 250.0;  // how far out behind us to find the closest car 1000

// priority levels for costs
const double COLLISION  = pow(10.0, 7);
const double DANGER     = pow(10.0, 5);
const double COMFORT    = pow(10.0, 3);
const double EFFICIENCY = pow(10.0, 1);

#endif /* CONFIG_H */
