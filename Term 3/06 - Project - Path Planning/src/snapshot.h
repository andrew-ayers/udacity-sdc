#ifndef SNAPSHOT_H
#define SNAPSHOT_H
#include <string>

using namespace std;

class Snapshot {
 public:
  int lane;
  double s;
  double v;
  double a;
  string state;

  /**
  * Constructor
  */
  Snapshot(int lane, double s, double v, double a, string state);

  /**
  * Destructor
  */
  virtual ~Snapshot();
};
#endif /* SNAPSHOT_H */
