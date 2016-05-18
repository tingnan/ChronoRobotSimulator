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
  ChFunctionPID(std::shared_ptr<ChFunction> motor_funct,
                std::shared_ptr<ChLinkEngine> engine)
      : motor_funct_(motor_funct), engine_(engine) {}
  ~ChFunctionPID() {}
  ChFunction *Clone() const { return new ChFunctionPID(motor_funct_, engine_); }
  int Get_Type() { return 9527; }
  double Get_y(double curr_t) const;
  double Get_y_dx(double curr_t) const { return 0; }

protected:
  double p_gain_ = 3.50;
  double i_gain_ = 0.00;
  double d_gain_ = 1e-3;
  double t_limit_ = 1.5;
  double a_limit_ = 1.2;

  double cum_error_ = 0;
  std::shared_ptr<ChFunction> motor_funct_;
  std::shared_ptr<ChLinkEngine> engine_;
};

class ChServoMotor {
public:
  explicit ChServoMotor(std::shared_ptr<ChLinkEngine> engine)
      : engine_(engine) {}
  void Initialize(std::shared_ptr<ChFunction> motor_funct,
                  ChLinkEngine::eCh_eng_mode mode_flag);
  int GetMotorID() { return engine_->GetIdentifier(); }
  double GetMotorRotation() { return engine_->Get_mot_rot(); }
  double GetMotorTorque() {
    if (motor_mode_ == ChLinkEngine::ENG_MODE_ROTATION) {
      return engine_->Get_react_torque().z;
    }
    return engine_->Get_mot_torque();
  }

private:
  // The underlying ChLinkEngine used.
  std::shared_ptr<ChLinkEngine> engine_;
  ChLinkEngine::eCh_eng_mode motor_mode_;
};

} // namespace chrono

#endif // INCLUDE_CHFUNCTION_CONTROLLER_H_
