#include "snapshot.h"
#include <string>

/**
 * Initializes Snapshot
 */
Snapshot::Snapshot(int lane, int s, int v, int a, string state) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  this->state = state;
}

Snapshot::~Snapshot() {}
