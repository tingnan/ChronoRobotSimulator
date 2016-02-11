#ifndef INCLUDE_MOTOR_FUNCTION_H_
#define INCLUDE_MOTOR_FUNCTION_H_

#include <motion_functions/ChFunction_Base.h>

namespace chrono {

// A nonlinear phased based sinusoidal function
class ChFunctionMotor : public ChFunction {
public:
  ChFunctionMotor(double amp, double freq, double initial_phase)
      : amplitude_(amp), frequency_(freq), phase_(initial_phase) {}
  ~ChFunctionMotor() {}
  ChFunction *new_Duplicate() {
    return new ChFunctionMotor(amplitude_, frequency_, phase_);
  }
  int Get_Type() { return 8607; }
  double Get_y(double t) { return amplitude_ * sin(phase_); }
  double Get_y_dx(double t) { return amplitude_ * frequency_ * cos(phase_); }
  void SetAmplitude(double amp) { amplitude_ = amp; }
  void SetFrequency(double freq) { frequency_ = freq; };
  void SetPhase(double phase) { phase_ = phase; }
  double GetAmplitude() { return amplitude_; }
  double GetFrequency() { return frequency_; }
  double GetCurrentPhase() { return phase_; }

protected:
  double amplitude_ = 0;
  double frequency_ = 0;
  double phase_ = 0;
};

} // namespace chrono

#endif // INCLUDE_MOTOR_FUNCTION_H_
