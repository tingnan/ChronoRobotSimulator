#ifndef INCLUDE_CHFUNCTION_CONTROLLER_H_
#define INCLUDE_CHFUNCTION_CONTROLLER_H_

#include <vector>

#include <motion_functions/ChFunction_Base.h>
#include <physics/ChSystem.h>
#include <physics/ChLinkEngine.h>

namespace chrono {
class ChFunctionController : public ChFunction {
public:
  ChFunctionController(ChSystem *ch_system, ChLinkEngine *engine,
                       std::vector<ChBody *> body_list,
                       std::vector<ChLinkEngine *> engine_list)
      : ch_system_(ch_system), body_list_(body_list), engine_(engine),
        engine_list_(engine_list) {}
  ~ChFunctionController() {}

  ChFunction *new_Duplicate() {
    return new ChFunctionController(ch_system_, engine_, body_list_,
                                    engine_list_);
  }

  int Get_Type() { return 9527; }

  double Get_y(double curr_t) { return GetPDTorque(curr_t); }

  double Get_y_dx(double new_t) {
    // it does not make sense to provide dtau/dt
    return 0;
  }

  double omega = 0.2 * CH_C_2PI;
  double amplitude = 0.5;
  double p_gain = 1.0;
  double d_gain = 0.0;

protected:
  // The low level PID controller in motor.
  double GetPDTorque(double t) {
    double desired_angle = GetPatternAngle(t);
    double desired_angular_speed = GetPatternAngularSpeed(t);
    double curr_angle = engine_->Get_mot_rot();
    double curr_angular_speed = engine_->Get_mot_rot_dt();
    double torque = p_gain * (desired_angle - curr_angle) +
                    d_gain * (desired_angular_speed - curr_angular_speed);
    torque = std::max(std::min(1.0, torque), -1.0);
    return torque;
  }

  double GetPatternAngle(double t) {
    const double phase =
        double(engine_->GetIdentifier() * 2) / engine_list_.size() * CH_C_2PI;
    double desired_angle = amplitude * sin(omega * t + phase);
    return desired_angle;
  }

  double GetPatternAngularSpeed(double t) {
    const double phase =
        double(engine_->GetIdentifier() * 2) / engine_list_.size() * CH_C_2PI;
    double desired_angular_speed = amplitude * omega * cos(omega * t + phase);
    return desired_angular_speed;
  }

  // The dynamical system.
  ChSystem *ch_system_;
  // The list of all the bodies needed.
  std::vector<ChBody *> body_list_;
  // The current engine that is controlled.
  ChLinkEngine *engine_;
  // The list of all the engines needed.
  std::vector<ChLinkEngine *> engine_list_;
};
} // namespace chrono

#endif // INCLUDE_CHFUNCTION_CONTROLLER_H_
