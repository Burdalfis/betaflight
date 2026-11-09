// DC-equivalent motor / static-prop load model, independent of Gazebo.
// SPDX-License-Identifier: GPL-3.0-or-later
// Equivalent-circuit equations: https://www.mathworks.com/help/simscape-electrical/ref/dcmotor.html
// PWM input / motor power distinction:
// https://support.maxongroup.com/hc/en-us/articles/360012273994
#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tricopter
{
struct MotorParameters
{
  // All electrical quantities use one consistent DC-equivalent convention.
  // Phase / line resistance and RMS currents must be converted before use.
  double kvRpmPerVolt {1800};
  // Terminal resistance. A balanced delta's individual branch is1.5 times
  // this value. With ideal two-terminal drive and a floating third terminal,
  // branch currents=(2I/3,I/3,I/3), giving sum(Rbranch*Ibranch²)=Rterminal*I².
  // The branch resistance must not replace Rterminal in that power expression.
  double resistanceOhm {.0815};
  double noLoadCurrentA {1.2};
  double noLoadVoltageV {10};
  double voltageV {22.2};
  // Explicit model current-regulator assumption, not an ESC hardware rating.
  double currentLimitA {35.22};
  // Averaged synchronous-PWM approximation: negative current retards an
  // overspeed rotor toward the requested voltage. Zero disables braking.
  // This default bound is an assumption, not identified ESC protection.
  double brakeCurrentLimitA {35.22};
  // Damped throttle reductions and the ESC's separate Brake On Stop option
  // are distinct. Invalid/disabled drive always coasts regardless of this.
  bool brakeOnStop {false};
  double rotorInertiaKgM2 {1e-5};
  double thrustCoefficientNPerRadS2 {0};
  double torqueCoefficientNmPerRadS2 {0};
  // Smoothed Coulomb loss is anchored to measured I0 at noLoadVoltageV.
  // One no-load point does not identify the real loss curve at other speeds.
  double frictionSmoothingRadS {20};

  double MotorConstant() const
  {
    constexpr double pi = 3.14159265358979323846;
    return 60 / (2*pi*kvRpmPerVolt); // Ke [V s/rad] = Kt [N m/A]
  }

  double NoLoadReferenceSpeedRadS() const
  {
    return (noLoadVoltageV-resistanceOhm*noLoadCurrentA)/MotorConstant();
  }

  double NoLoadLossCurrentA() const
  {
    // Normalize the smooth loss so its equivalent loss current is exactlyI0
    // at the reference speed, rather than merely approachingI0 asymptotically.
    const double reference = NoLoadReferenceSpeedRadS();
    return noLoadCurrentA*(reference+frictionSmoothingRadS)/reference;
  }

  void Validate() const
  {
    for (double value : {kvRpmPerVolt, resistanceOhm, voltageV, noLoadVoltageV,
                        currentLimitA, rotorInertiaKgM2, frictionSmoothingRadS})
      if (!std::isfinite(value) || value <= 0)
        throw std::invalid_argument("Motor coefficient must be positive and finite");
    for (double value : {noLoadCurrentA, brakeCurrentLimitA, thrustCoefficientNPerRadS2,
                        torqueCoefficientNmPerRadS2})
      if (!std::isfinite(value) || value < 0)
        throw std::invalid_argument("Motor load coefficient must be nonnegative and finite");
    if (noLoadVoltageV <= resistanceOhm*noLoadCurrentA)
      throw std::invalid_argument("No-load reference voltage must exceed its terminal voltage drop");
  }
};

struct MotorStep
{
  double omegaRadS {0};
  double alphaRadS2 {0};
  double terminalCurrentA {0};
  double batteryCurrentA {0};
  double effectiveVoltageV {0};
  double copperPowerW {0};
  double electromagneticTorqueNm {0};
  double lossTorqueNm {0};
  double shaftTorqueNm {0};
  double propTorqueNm {0};
  double thrustN {0};
  // Backward Euler's nonnegative discrete energy loss is not physical heat.
  double numericalDissipationW {0};
};

namespace motor_detail
{
inline MotorStep AtSpeed(const MotorParameters &p, double omega, double duty, bool driveEnabled = true)
{
  MotorStep result;
  result.omegaRadS = omega;
  const double constant = p.MotorConstant();
  const double requestedVoltage = duty*p.voltageV;
  // A synchronous bridge can sink terminal current when back-EMF exceeds the
  // requested voltage. Missing drive is open circuit, not a brake command.
  const bool conducting = driveEnabled && (duty > 0 || p.brakeOnStop);
  result.terminalCurrentA = conducting ? std::clamp(
      (requestedVoltage-constant*omega)/p.resistanceOhm,
      -p.brakeCurrentLimitA, p.currentLimitA) : 0;
  result.electromagneticTorqueNm = constant*result.terminalCurrentA;
  result.lossTorqueNm = constant*p.NoLoadLossCurrentA() *
      omega/(omega+p.frictionSmoothingRadS);
  result.shaftTorqueNm = result.electromagneticTorqueNm-result.lossTorqueNm;
  result.propTorqueNm = p.torqueCoefficientNmPerRadS2*omega*omega;
  result.thrustN = p.thrustCoefficientNPerRadS2*omega*omega;
  result.copperPowerW = p.resistanceOhm*result.terminalCurrentA*result.terminalCurrentA;
  // Current limiting lowers effective voltage in drive and raises it in
  // braking. Retain signed power: the ideal fixed-voltage bus absorbs returned
  // energy. At unclipped zero-duty short braking Veff=0 and copper takes it.
  if (result.terminalCurrentA != 0)
    result.effectiveVoltageV = std::max(0.0,
        constant*omega+p.resistanceOhm*result.terminalCurrentA);
  result.batteryCurrentA = result.effectiveVoltageV*result.terminalCurrentA/p.voltageV;
  return result;
}

inline double Command(double duty)
{
  if (!std::isfinite(duty)) throw std::invalid_argument("Nonfinite motor duty");
  return std::clamp(duty, 0.0, 1.0);
}
}

inline MotorStep StepMotor(const MotorParameters &p, double previousOmega,
                           double duty, double dt, bool driveEnabled = true)
{
  p.Validate();
  duty = motor_detail::Command(duty);
  if (!std::isfinite(previousOmega) || previousOmega < 0 ||
      !std::isfinite(dt) || dt < 0)
    throw std::invalid_argument("Invalid motor speed or timestep");
  if (dt == 0) return motor_detail::AtSpeed(p, previousOmega, duty, driveEnabled);

  // Quasi-static electrical current; implicit mechanical integration. The
  // residual is strictly increasing, even across the current clamp, so this
  // brackets one nonnegative solution without a stiff explicit speed update.
  double lower = 0;
  double upper = std::max(previousOmega, duty*p.voltageV/p.MotorConstant());
  for (unsigned iteration = 0; iteration < 64; ++iteration)
  {
    const double omega = (lower+upper)/2;
    const auto point = motor_detail::AtSpeed(p, omega, duty, driveEnabled);
    const double residual = p.rotorInertiaKgM2*(omega-previousOmega)/dt -
                            point.shaftTorqueNm+point.propTorqueNm;
    if (residual > 0) upper = omega;
    else lower = omega;
  }
  auto result = motor_detail::AtSpeed(p, (lower+upper)/2, duty, driveEnabled);
  const double delta = result.omegaRadS-previousOmega;
  result.alphaRadS2 = delta/dt;
  result.numericalDissipationW = .5*p.rotorInertiaKgM2*delta*delta/dt;
  return result;
}

inline MotorStep MotorEquilibrium(const MotorParameters &p, double duty)
{
  p.Validate();
  duty = motor_detail::Command(duty);
  double lower = 0;
  double upper = duty*p.voltageV/p.MotorConstant();
  for (unsigned iteration = 0; iteration < 64; ++iteration)
  {
    const double omega = (lower+upper)/2;
    const auto point = motor_detail::AtSpeed(p, omega, duty);
    if (point.shaftTorqueNm-point.propTorqueNm > 0) lower = omega;
    else upper = omega;
  }
  return motor_detail::AtSpeed(p, (lower+upper)/2, duty);
}
}
