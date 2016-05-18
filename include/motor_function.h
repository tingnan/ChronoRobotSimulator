#ifndef INCLUDE_MOTOR_FUNCTION_H_
#define INCLUDE_MOTOR_FUNCTION_H_

#include <motion_functions/ChFunction_Base.h>

namespace chrono {

// A nonlinear phased based sinusoidal function
class ChFunctionMotor : public ChFunction {
public:
  ChFunctionMotor(double y, double y_dt) : y_(y), y_dt_(y_dt) {}
  ~ChFunctionMotor() {}
  ChFunction *Clone() const { return new ChFunctionMotor(y_, y_dt_); }
  int Get_Type() { return 8607; }
  double Get_y(double t) const { return y_; }
  double Get_y_dx(double t) const { return y_dt_; }
  void SetAngle(double th) { y_ = th; }
  void SetAngleDt(double th_dt) { y_dt_ = th_dt; }

protected:
  double y_;
  double y_dt_;
};

} // namespace chrono

#endif // INCLUDE_MOTOR_FUNCTION_H_
