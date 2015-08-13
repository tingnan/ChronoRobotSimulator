#ifndef INCLUDE_CHFUNCTION_SQUAREWAVE_H_
#define INCLUDE_CHFUNCTION_SQUAREWAVE_H_

#include <cmath>

#include <motion_functions/ChFunction_Base.h>

inline double matlabmod(double x, double y) {
  int n = floor(x / y);
  double m = x - n * y;
  return m;
}

namespace chrono {
class ChFunction_SquareWave : public ChFunction {
public:
  ChFunction_SquareWave() {
    ratio_ = 0.5;
    period_ = 1;
    value_ = 1;
    offset_ = 0;
  }
  ChFunction_SquareWave(double r, double p, double v, double o)
      : ratio_(r), period_(p), value_(v), offset_(o) {}
  ~ChFunction_SquareWave() {}

  ChFunction *new_Duplicate() {
    return new ChFunction_SquareWave(ratio_, period_, value_, offset_);
  }
  int Get_Type() { return 9527; }
  double Get_y(double x) {
    // warning, fmod give negative value for negative x
    double ip = matlabmod(x - offset_, period_) / period_;
    if (ip < ratio_) {
      // it is on
      return value_;
    } else
      return 0;
  }
  double Get_y_dx(double x) { return 0; }

protected:
  // the "ON" ratio;
  double ratio_;
  // the period of cycle
  double period_;
  // the
  double value_;
  double offset_;
};
} // namespace chrono

#endif // INCLUDE_CHFUNCTION_SQUAREWAVE_H_
