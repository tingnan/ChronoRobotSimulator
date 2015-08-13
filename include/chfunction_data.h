#ifndef INCLUDE_CHFUNCTION_DATA_H_
#define INCLUDE_CHFUNCTION_DATA_H_

#include <cmath>
#include <vector>
#include <algorithm>

#include <motion_functions/ChFunction_Base.h>

namespace chrono {
class ChFunction_Data : public ChFunction {
public:
  ChFunction_Data() {}
  ChFunction_Data(const std::vector<double> &t, const std::vector<double> &v)
      : time_(t), data_(v) {}
  ~ChFunction_Data() {}

  ChFunction *new_Duplicate() { return new ChFunction_Data(time_, data_); }
  int Get_Type() { return 1111; }
  double Get_y(double t) {
    // we are going to do the interpolation;
    if (time_.size() <= 1)
      return 0;
    // the first element in time_ that is greater than t;
    std::vector<double>::iterator itr =
        upper_bound(time_.begin(), time_.end(), t);
    if (itr == time_.begin() || itr == time_.end()) {
      return 0;
    }

    int idx = (itr - time_.begin());
    assert(idx == 0 || idx == time_.size());
    double t1 = time_[idx - 1];
    double y1 = data_[idx - 1];
    double t2 = time_[idx];
    double y2 = data_[idx];
    double w1 = (t2 - t) / (t2 - t1);
    double w2 = (t - t1) / (t2 - t1);

    return y1 * w1 + y2 * w2;
  }
  double Get_y_dx(double t) { return 0; }

protected:
  // the time interval for the
  // must be a sorted, monotonically increasing array or the results will blow
  // up
  std::vector<double> time_;
  // the actual data at the corrsponding time;
  std::vector<double> data_;
};
} // namespace chrono

#endif // INCLUDE_CHFUNCTION_DATA_H_
