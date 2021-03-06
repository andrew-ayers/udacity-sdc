#ifndef CONFIG_H
#define CONFIG_H

using namespace std;

// general settings (DO NOT TOUCH)
const bool DEBUG = true;

const double MPH_TO_MPS = 2.24;     // MPH to meters-per-second, 2.24mph -> 1 m/s
const double SPLINE_SPACING = 30;   // 30 meters between spline segments
const int MAX_TICKS = 30;           // number of simulator "ticks" before behavior planner processing for ego
const double SECS_PER_TICK = 0.02;  // number of elapsed seconds per "tick" of simulator
const int FILLER_DIST = 50;         // 50 meters

// ego vehicle settings
const int EGO_START_LANE = 1;       // lane the ego vehicle starts in (fixed in simulator)
const double EGO_MAX_ACCEL = 22.0;  // 22 meters-per-second acceleration value
const double EGO_MAX_DECEL = 11.0;  // 11 meters-per-second decceleration value
const double EGO_MAX_VELOCITY = 49.75;

// road settings
const int NUM_LANES = 3;

// horizon settings
const double PREFERRED_BUFFER = 25;  // 0.5s buffer (25 * 0.02) - impacts "keep lane" behavior.

const int PREDICTIONS_HZ = 75;  // 1.5s horizon (75 * 0.02)
const int TRAJECTORIES_HZ = 75;

const double LEADING_HZ = 150.0;  // how far out directly ahead of us to find the closest car (3s horizon - 150 * 0.02)
const double FOLLOW_HZ = 175.0;   // how far out behind us to find the closest car (3.5s horizon - 175 * 0.02)

// priority levels for costs
const double COLLISION  = pow(10.0, 7);
const double DANGER     = pow(10.0, 6);
const double EFFICIENCY = pow(10.5, 4);  // prioritize efficiency just slightly over comfort
const double COMFORT    = pow(10.0, 4);

#endif /* CONFIG_H */
