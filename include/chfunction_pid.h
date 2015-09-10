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

    double curr_rotation = engine_->Get_mot_rot();
    double desired_rotation = engine_->Get_rot_funct()->Get_y(curr_t);
    double curr_error = desired_rotation - curr_rotation;

    double curr_rotation_dt = engine_->Get_mot_rot_dt();
    double desired_rotation_dt = engine_->Get_rot_funct()->Get_y_dx(curr_t);
    double curr_error_dt = desired_rotation_dt - curr_rotation_dt;

    // trapezoidal method
    accu_error_ += 0.5 * (curr_error + last_error_) * dt;

    double output = p_coeff * curr_error + i_coeff * accu_error_ +
                    d_coeff * (curr_error - last_error_) / dt;
    if (max > min) {
      // set the limit
      output = std::max(std::min(output, max), min);
    }

    last_error_ = curr_error;
    last_called_ = curr_t;
    last_value_ = output;
    return output;
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
  // last outputput
  double last_value_;
};
} // namespace chrono

#endif // INCLUDE_CHFUNCTION_PID_H_
