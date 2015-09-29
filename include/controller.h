#ifndef INCLUDE_CONTROLLER_H_
#define INCLUDE_CONTROLLER_H_

#include <vector>

#include <motion_functions/ChFunction_Base.h>
#include <physics/ChSystem.h>
#include <physics/ChLinkEngine.h>
#include <physics/ChContactContainerBase.h>

class Controller {
public:
  Controller(chrono::ChSystem *ch_system, class Robot *i_robot);
  // Step the controller
  void Step(double dt);
  // get the toruqe for joint i
  size_t GetNumEngines();
  chrono::ChLinkEngine *GetEngine(size_t i);
  double GetExtTorque(size_t i, double t);
  double GetExtTorqueWeight(size_t i, double t);
  double GetPatternAngle(size_t i, double t);
  double GetPatternAngularSpeed(size_t i, double t);

private:
  chrono::ChSystem *ch_system_;
  class Robot *robot_;
  // Contact force on each of the robot segment.
  chrono::ChReportContactCallback2 *contact_reporter_;
  std::vector<chrono::ChVector<>> contact_force_list_;
  // the torques at joints, computed from contact forces.
  chrono::ChVectorDynamic<> torques_ext_;
  chrono::ChVectorDynamic<> weight_;
  // Parametr for the CPG
  double omega_ = 0.2 * chrono::CH_C_2PI;
  double amplitude_ = 0.6;
};

void UseController(Controller *controller);
void UsePositionControl(Robot *robot);

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

  double Get_y(double curr_t) {
    double torque = ComputeDriveTorque(curr_t) + ComputeLimitTorque(curr_t);
    torque = std::max(std::min(torque_limit, torque), -torque_limit);
    return torque;
  }

  double Get_y_dx(double new_t) {
    // it does not make sense to provide dtau/dt
    return 0;
  }

  double p_gain = 0.50;
  double i_gain = 0.00;
  double d_gain = 0.00;

  double torque_limit = 0.5;

protected:
  double ComputeAmpMod(double t) {
    return controller_->GetExtTorqueWeight(index_ + 1, t);
  }

  double ComputeExternalTorque(double t) {
    return controller_->GetExtTorque(index_ + 1, t);
  }

  // The low level PID controller in motor.
  double ComputeDriveTorque(double t) {
    double amp_mod = ComputeAmpMod(t);
    double desired_angle = amp_mod * controller_->GetPatternAngle(index_, t);
    double desired_angular_speed =
        amp_mod * controller_->GetPatternAngularSpeed(index_, t);
    double curr_angle = controller_->GetEngine(index_)->Get_mot_rot();
    double curr_angular_speed =
        controller_->GetEngine(index_)->Get_mot_rot_dt();
    // std::cout << index_ << " " << desired_angle - curr_angle << std::endl;
    cum_error_ += desired_angle - curr_angle;
    double torque = p_gain * (desired_angle - curr_angle) +
                    d_gain * (desired_angular_speed - curr_angular_speed) +
                    cum_error_ * i_gain;

    if (desired_angular_speed < 0) {
      torque = curr_angle > desired_angle
                   ? p_gain * (desired_angle - curr_angle)
                   : 0;
    } else {
      torque = curr_angle < desired_angle
                   ? p_gain * (desired_angle - curr_angle)
                   : 0;
    }

    return torque;
  }

  double ComputeLimitTorque(double t) {
    auto engine = controller_->GetEngine(index_);
    double curr_angle = engine->Get_mot_rot();
    double angle_limit = 1.2;
    double angle_limit_compliance = 1;
    if (curr_angle > angle_limit) {
      return -angle_limit_compliance * (curr_angle - angle_limit);
    }
    if (curr_angle < -angle_limit) {
      return -angle_limit_compliance * (curr_angle - angle_limit);
    }
    return 0;
  }

  double cum_error_ = 0;

  Controller *controller_;
  size_t index_;
};
} // namespace chrono

#endif // INCLUDE_CHFUNCTION_CONTROLLER_H_
