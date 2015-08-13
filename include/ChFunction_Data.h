#pragma once
#include "motion_functions/ChFunction_Base.h"
#include <cmath>
#include <vector>
#include <algorithm>
namespace chrono {
class ChFunction_Data : public ChFunction {
protected:
  // the time interval for the
  // must be a sorted, monotonically increasing array or the results will blow
  // up
  std::vector<double> mTime;
  // the actual data at the corrsponding time;
  std::vector<double> mData;

public:
  ChFunction_Data() {}
  ChFunction_Data(const std::vector<double> &t, const std::vector<double> &v)
      : mTime(t), mData(v) {}
  ~ChFunction_Data() {}

  ChFunction *new_Duplicate() { return new ChFunction_Data(mTime, mData); }
  int Get_Type() { return 1111; }
  double Get_y(double t) {
    // we are going to do the interpolation;
    if (mTime.size() <= 1)
      return 0;
    // the first element in mTime that is greater than t;
    std::vector<double>::iterator itr =
        upper_bound(mTime.begin(), mTime.end(), t);
    if (itr == mTime.begin() || itr == mTime.end()) {
      return 0;
    }

    int idx = (itr - mTime.begin());
    assert(idx == 0 || idx == mTime.size());
    double t1 = mTime[idx - 1];
    double y1 = mData[idx - 1];
    double t2 = mTime[idx];
    double y2 = mData[idx];
    double w1 = (t2 - t) / (t2 - t1);
    double w2 = (t - t1) / (t2 - t1);

    return y1 * w1 + y2 * w2;
  }
  double Get_y_dx(double t) { return 0; }
};
}
