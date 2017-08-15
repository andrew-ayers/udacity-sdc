#ifndef SNAPSHOT_H
#define SNAPSHOT_H
#include <string>

using namespace std;

class Snapshot {
 public:
  int lane;
  int s;
  int v;
  int a;
  string state;

  /**
  * Constructor
  */
  Snapshot(int lane, int s, int v, int a, string state);

  /**
  * Destructor
  */
  virtual ~Snapshot();
};
#endif /* SNAPSHOT_H */
