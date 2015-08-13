#pragma once
#include "motion_functions/ChFunction_Base.h"
#include <cmath>
#include "physics/ChLinkEngine.h"
#include <algorithm>

namespace chrono {
class ChFunction_PID : public ChFunction {
protected:
  // last input
  double mLastError;
  // accumulated input
  double mAccuError;
  // last time the function is called
  double mLastCalled;
  // the engine it is attached to
  ChLinkEngine *mEngine;
  // last output
  double mLastValue;

public:
  double P;
  double I;
  double D;
  double Max;
  double Min;
  double mDt;

  ChFunction_PID()
      : P(1.0), I(0.0), D(0.0), Max(0.0), Min(0.0), mLastError(0.0),
        mAccuError(0.0), mLastCalled(0.0), mEngine(NULL), mDt(0.0),
        mLastValue(0.0) {}
  ChFunction_PID(double p, double i, double d, ChLinkEngine *e)
      : P(p), I(i), D(d), Max(0.0), Min(0.0), mLastError(0.0), mAccuError(0.0),
        mLastCalled(0.0), mEngine(e), mDt(0.0), mLastValue(0.0) {}
  ~ChFunction_PID() {}

  ChFunction *new_Duplicate() { return new ChFunction_PID(P, I, D, NULL); }

  int Get_Type() { return 2333; }

  double Get_y(double curr_t) {
    double dt = curr_t - mLastCalled;
    if (dt < mDt)
      return mLastValue;
    // we will use the internal information of the mEngine to compute the error;
    // then we can use the error to compute the torque needed

    double currRotation = mEngine->Get_mot_rot();
    double desiredRotation = mEngine->Get_rot_funct()->Get_y(curr_t);
    double currError = desiredRotation - currRotation;

    double currRotation_dt = mEngine->Get_mot_rot_dt();
    double desiredRotation_dt = mEngine->Get_rot_funct()->Get_y_dx(curr_t);
    double currError_dt = desiredRotation_dt - currRotation_dt;

    // trapezoidal method
    mAccuError += 0.5 * (currError + mLastError) * dt;

    double Out =
        P * currError + I * mAccuError + D * (currError - mLastError) / dt;
    if (Max > Min) {
      // set the limit
      Out = std::max(std::min(Out, Max), Min);
    }

    mLastError = currError;
    mLastCalled = curr_t;
    mLastValue = Out;
    // std::cout << mAccuError << std::endl;
    return Out;
  }

  double Get_y_dx(double new_t) {
    // it does not make sense to provide dtau/dt
    return 0;
  }

  void SetMax(double max) { Max = max; }
  void SetMin(double min) { Min = min; }
};
}
