#ifndef INCLUDE_CHFUNCTION_CONTROLLER_H_
#define INCLUDE_CHFUNCTION_CONTROLLER_H_

#include <cmath>
#include <algorithm>

#include <motion_functions/ChFunction_Base.h>

class Controller;

namespace chrono {
class ChFunctionController : public ChFunction {
public:
  ChFunctionController(size_t index, Controller *controller)
      : index_(index), controller_(controller) {}
  ~ChFunctionController() {}
  ChFunction *new_Duplicate() {
    return new ChFunctionController(index_, controller_);
  }
  int Get_Type() { return 9527; }
  double Get_y(double curr_t);
  double Get_y_dx(double new_t) { return 0; }

  double p_gain = 3.40;
  double i_gain = 0.00;
  double d_gain = 1e-3;
  double torque_limit = 1.5;
  double angle_limit = 1.2;

protected:
  double GetMediaTorque(double t);
  // The low level PID controller in motor.
  double ComputeDriveTorque(double t);
  double cum_error_ = 0;
  Controller *controller_;
  size_t index_;
};
} // namespace chrono

#endif // INCLUDE_CHFUNCTION_CONTROLLER_H_
