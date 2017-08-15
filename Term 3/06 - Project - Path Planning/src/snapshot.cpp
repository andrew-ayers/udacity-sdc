#include "snapshot.h"
#include <string>

/**
 * Initializes Snapshot
 */
Snapshot::Snapshot(int lane, double s, double v, double a, string state) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  this->state = state;
}

Snapshot::~Snapshot() {}
