#include <cassert>
#include <cmath>
#include "../src/calibration.h"

static bool near(float a, float b, float eps=0.01f) { return std::fabs(a-b) < eps; }

int main() {
  calibration::Wheel w;
  w.sign = 1;
  assert(calibration::computeLine(60, 2000, 5000, w));
  assert(near(w.b, 40.0f/3000.0f));
  assert(near(w.a, 60.0f - (40.0f/3000.0f)*2000.0f));
  assert(near(w.v100, 5000.0f));
  assert(w.sign == 1);

  calibration::Wheel bad;
  assert(!calibration::computeLine(60, 5000, 5000, bad)); // v100<=v60
  assert(!calibration::computeLine(60, 0, 100, bad));     // v100<200
  assert(!calibration::computeLine(60, 4000, 5000, bad)); // a<0
  assert(!calibration::computeLine(99, 1, 2000, bad));    // d1 非法

  calibration::Wheel wheels[4] = {};
  for (int i=0;i<4;++i) wheels[i].vsim = 4000 + i*500;
  assert(near(calibration::computeVmax(wheels, 80), 3200.0f));
  return 0;
}
