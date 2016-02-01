#ifndef INCLUDE_CHFUNCTION_CONTROLLER_H_
#define INCLUDE_CHFUNCTION_CONTROLLER_H_

#include <algorithm>
#include <cmath>
#include <motion_functions/ChFunction_Base.h>
#include <physics/ChLinkEngine.h>

class Controller;

namespace chrono {

class ChFunctionPID : public ChFunction {
public:
  ChFunctionPID(ChSharedPtr<ChFunction> motor_funct,
                ChSharedPtr<ChLinkEngine> engine)
      : motor_funct_(motor_funct), engine_(engine) {}
  ~ChFunctionPID() {}
  ChFunction *new_Duplicate() {
    return new ChFunctionPID(motor_funct_, engine_);
  }
  int Get_Type() { return 9527; }
  double Get_y(double curr_t);
  double Get_y_dx(double curr_t) { return 0; }

protected:
  double p_gain_ = 3.40;
  double i_gain_ = 0.00;
  double d_gain_ = 1e-3;
  double t_limit_ = 1.5;
  double a_limit_ = 1.2;

  double cum_error_ = 0;
  ChSharedPtr<ChFunction> motor_funct_;
  ChSharedPtr<ChLinkEngine> engine_;
};

class ChServoMotor : public ChShared {
public:
  explicit ChServoMotor(ChSharedPtr<ChLinkEngine> engine) : engine_(engine) {}
  void Initialize(ChSharedPtr<ChFunction> motor_funct,
                  ChLinkEngine::eCh_eng_mode mode_flag);
  void Serialize();

private:
  // The underlying ChLinkEngine used.
  ChSharedPtr<ChLinkEngine> engine_;
  ChLinkEngine::eCh_eng_mode motor_mode_;
};

} // namespace chrono

#endif // INCLUDE_CHFUNCTION_CONTROLLER_H_
