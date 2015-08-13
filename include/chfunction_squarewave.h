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
protected:
  // the "ON" ratio;
  double mRatio;
  // the period of cycle
  double mPeriod;
  // the
  double mValue;
  double mOffset;

public:
  ChFunction_SquareWave() {
    mRatio = 0.5;
    mPeriod = 1;
    mValue = 1;
    mOffset = 0;
  }
  ChFunction_SquareWave(double r, double p, double v, double o)
      : mRatio(r), mPeriod(p), mValue(v), mOffset(o) {}
  ~ChFunction_SquareWave() {}

  ChFunction *new_Duplicate() {
    return new ChFunction_SquareWave(mRatio, mPeriod, mValue, mOffset);
  }
  int Get_Type() { return 9527; }
  double Get_y(double x) {
    // warning, fmod give negative value for negative x
    double ip = matlabmod(x - mOffset, mPeriod) / mPeriod;
    if (ip < mRatio) {
      // it is on
      return mValue;
    } else
      return 0;
  }
  double Get_y_dx(double x) { return 0; }
};
}
#endif // INCLUDE_CHFUNCTION_SQUAREWAVE_H_
