#ifndef INCLUDE_CHFUNCTION_PID_H_
#define INCLUDE_CHFUNCTION_PID_H_

#include <cmath>
#include <algorithm>

#include <motion_functions/ChFunction_Base.h>
#include <physics/ChLinkEngine.h>

namespace chrono {
class ChFunction_PID : public ChFunction {
public:
  double p_coeff;
  double i_coeff;
  double d_coeff;
  double max;
  double min;
  double dt;

  ChFunction_PID()
      : p_coeff(1.0), i_coeff(0.0), d_coeff(0.0), max(0.0), min(0.0),
        last_error_(0.0), accu_error_(0.0), last_called_(0.0), engine_(NULL),
        dt(0.0), last_value_(0.0) {}
  ChFunction_PID(double p, double i, double d, ChLinkEngine *e)
      : p_coeff(p), i_coeff(i), d_coeff(d), max(0.0), min(0.0),
        last_error_(0.0), accu_error_(0.0), last_called_(0.0), engine_(e),
        dt(0.0), last_value_(0.0) {}
  ~ChFunction_PID() {}

  ChFunction *new_Duplicate() {
    return new ChFunction_PID(p_coeff, i_coeff, d_coeff, NULL);
  }

  int Get_Type() { return 2333; }

  double Get_y(double curr_t) {
    double dt = curr_t - last_called_;
    if (dt < dt)
      return last_value_;
    // we will use the internal information of the engine_ to compute the error;
    // then we can use the error to compute the torque needed

    double currRotation = engine_->Get_mot_rot();
    double desiredRotation = engine_->Get_rot_funct()->Get_y(curr_t);
    double currError = desiredRotation - currRotation;

    double currRotation_dt = engine_->Get_mot_rot_dt();
    double desiredRotation_dt = engine_->Get_rot_funct()->Get_y_dx(curr_t);
    double currError_dt = desiredRotation_dt - currRotation_dt;

    // trapezoidal method
    accu_error_ += 0.5 * (currError + last_error_) * dt;

    double Out = p_coeff * currError + i_coeff * accu_error_ +
                 d_coeff * (currError - last_error_) / dt;
    if (max > min) {
      // set the limit
      Out = std::max(std::min(Out, max), min);
    }

    last_error_ = currError;
    last_called_ = curr_t;
    last_value_ = Out;
    // std::cout << accu_error_ << std::endl;
    return Out;
  }

  double Get_y_dx(double new_t) {
    // it does not make sense to provide dtau/dt
    return 0;
  }

  void SetMax(double max) { max = max; }
  void SetMin(double min) { min = min; }

protected:
  // last input
  double last_error_;
  // accumulated input
  double accu_error_;
  // last time the function is called
  double last_called_;
  // the engine it is attached to
  ChLinkEngine *engine_;
  // last output
  double last_value_;
};
} // namespace chrono

#endif // INCLUDE_CHFUNCTION_PID_H_
