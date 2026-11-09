// Focused torque, energy and numerical checks for the DC-equivalent plant.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "MotorModel.hh"
#include "TricopterMath.hh"

#include <iostream>
#include <limits>
#include <string>

namespace
{
unsigned checks = 0;
void Check(bool result, const std::string &what)
{
  ++checks;
  if (!result) throw std::runtime_error(what);
}
void Near(double actual, double expected, const std::string &what, double tolerance = 1e-9)
{
  Check(std::isfinite(actual) && std::abs(actual-expected) <= tolerance, what);
}
}

int main()
{
  try
  {
    using namespace tricopter;
    MotorParameters p;
    p.rotorInertiaKgM2 = 6e-6;
    // Test coefficients are independent positive loads, not manufacturer fits.
    p.thrustCoefficientNPerRadS2 = 1.2e-6;
    p.torqueCoefficientNmPerRadS2 = 2e-8;
    const double k = p.MotorConstant();
    Near(k, 0.005305164769729845, "1800KV SI conversion", 1e-15);

    // Closed-form steady solution without motor friction or current clipping.
    auto simple = p;
    simple.noLoadCurrentA = 0;
    simple.currentLimitA = 1000;
    const double duty = .6;
    const double a = k*duty*p.voltageV/p.resistanceOhm;
    const double b = k*k/p.resistanceOhm;
    const double c = p.torqueCoefficientNmPerRadS2;
    const double analytic = 2*a/(b+std::sqrt(b*b+4*c*a));
    const auto equilibrium = MotorEquilibrium(simple, duty);
    Near(equilibrium.omegaRadS, analytic, "Steady RPM matches quadratic load solution");
    Near(equilibrium.shaftTorqueNm, equilibrium.propTorqueNm, "Steady torque closes", 1e-13);

    // The specified smoothed I0 loss also has an independent no-prop solution.
    auto noProp = p;
    noProp.currentLimitA = 1000;
    noProp.torqueCoefficientNmPerRadS2 = 0;
    const double u = duty*p.voltageV;
    const double q = u-k*p.frictionSmoothingRadS-p.resistanceOhm*p.NoLoadLossCurrentA();
    const double noPropAnalytic = (q+std::sqrt(q*q+4*k*u*p.frictionSmoothingRadS))/(2*k);
    Near(MotorEquilibrium(noProp, duty).omegaRadS, noPropAnalytic,
         "No-load friction equilibrium matches independent quadratic");

    // The supplied1.2A at10V is an exact no-load calibration anchor. The
    // smooth low-speed regularization must not shift that reference point.
    auto reference = noProp;
    reference.voltageV = reference.noLoadVoltageV;
    const auto noLoad = MotorEquilibrium(reference, 1);
    Near(noLoad.omegaRadS, (10-.0815*1.2)/k, "Reference no-load RPM includes copper voltage drop");
    Near(noLoad.terminalCurrentA, 1.2, "Measured no-load current is reproduced exactly", 1e-12);
    Near(noLoad.batteryCurrentA, 1.2, "Reference full-duty battery current matches terminal current", 1e-12);
    Near(noLoad.shaftTorqueNm, 0, "No-load electromagnetic and friction torques balance", 1e-13);
    Near(reference.voltageV*noLoad.batteryCurrentA,
         noLoad.copperPowerW+noLoad.lossTorqueNm*noLoad.omegaRadS,
         "Reference no-load electrical power closes", 1e-10);
    // At an ideal delta's two driven terminals, the third node floats.
    // Ibranch=(2I/3,I/3,I/3), so phase copper sums to Rterminal*Iterminal².
    const double terminalCurrent = 12;
    const double branchResistance = 1.5*p.resistanceOhm;
    Near(branchResistance*(std::pow(2*terminalCurrent/3,2)+
         2*std::pow(terminalCurrent/3,2)), p.resistanceOhm*terminalCurrent*terminalCurrent,
         "Delta branch conversion preserves terminal copper power", 1e-13);

    double omega = 0;
    for (unsigned step = 0; step < 1000; ++step)
    {
      const double previous = omega;
      const auto result = StepMotor(p, previous, .7, .001);
      omega = result.omegaRadS;
      Check(omega >= previous && omega <= .7*p.voltageV/k, "Startup is monotone and bounded");
      Check(result.terminalCurrentA >= 0 && result.terminalCurrentA <= p.currentLimitA,
            "Current clamp is respected");
      Check(result.effectiveVoltageV <= .7*p.voltageV+1e-12,
            "Current regulator does not exceed requested voltage");
      Near(p.rotorInertiaKgM2*result.alphaRadS2,
           result.shaftTorqueNm-result.propTorqueNm, "Discrete angular momentum closes", 1e-12);
      const double inputPower = p.voltageV*result.batteryCurrentA;
      const double energyChange = .5*p.rotorInertiaKgM2*(omega*omega-previous*previous)/.001;
      Near(inputPower, result.copperPowerW + result.lossTorqueNm*omega +
           result.propTorqueNm*omega + energyChange + result.numericalDissipationW,
           "Input power closes with explicit implicit-Euler dissipation", 1e-8);
      Near(result.thrustN, p.thrustCoefficientNPerRadS2*omega*omega, "Thrust uses actual RPM");
      Near(result.propTorqueNm, p.torqueCoefficientNmPerRadS2*omega*omega,
           "Prop load uses the same actual RPM", 1e-12);
      // Aerodynamic torque and the existing relative-rotor inertial reaction
      // already produce the carrier motor torque. Do not add -Kt*I again.
      for (double spin : {-1., 1.})
      {
        const auto reaction = RotorReaction(p.rotorInertiaKgM2, spin*omega,
          spin*result.alphaRadS2, {0,0,1}, {0,0,0}).Total();
        Near(-spin*result.propTorqueNm+reaction[2], -spin*result.shaftTorqueNm,
             "Carrier motor torque is applied once", 1e-12);
      }
    }
    Near(omega, MotorEquilibrium(p, .7).omegaRadS, "Startup reaches torque equilibrium", 1e-8);

    const auto startup = StepMotor(p, 0, 1, .001);
    Near(startup.terminalCurrentA, p.currentLimitA, "Startup reaches configured current limit");
    Check(startup.effectiveVoltageV < p.voltageV, "Current limiting reduces effective PWM voltage");
    Check(startup.batteryCurrentA < startup.terminalCurrentA,
          "Battery current differs from terminal current under PWM");

    const double beforeCoast = omega;
    for (unsigned step = 0; step < 1000; ++step)
    {
      const auto result = StepMotor(p, omega, 0, .001);
      Check(result.omegaRadS >= 0 && result.omegaRadS <= omega, "Unpowered rotor coasts monotonically");
      Near(result.terminalCurrentA, 0, "Coast has no terminal drive current");
      Near(result.batteryCurrentA, 0, "Coast does not invent battery regeneration");
      omega = result.omegaRadS;
    }
    Check(omega < beforeCoast*.001, "Prop and internal losses dissipate rotor energy");
    Near(StepMotor(p, 0, 0, 1).omegaRadS, 0, "Stopped rotor remains stopped");
    Near(StepMotor(p, 100, .5, 0).omegaRadS, 100, "Zero timestep retains speed");
    auto coast = p;
    coast.brakeCurrentLimitA = 0;
    const double highSpeed = MotorEquilibrium(p, .7).omegaRadS;
    const auto braking = StepMotor(p, highSpeed, .3, .001);
    const auto unbraked = StepMotor(coast, highSpeed, .3, .001);
    Check(braking.terminalCurrentA < 0, "Default positive-duty reduction actively brakes");
    Check(braking.batteryCurrentA < 0, "Braking returns signed power to ideal absorbing bus");
    Check(braking.omegaRadS < unbraked.omegaRadS, "Braking reduces speed faster than coast");
    Near(braking.terminalCurrentA, -p.brakeCurrentLimitA, "Negative current has its own bound");
    Check(braking.effectiveVoltageV > .3*p.voltageV,
          "Current-limited braking raises effective voltage above requested duty");
    Near(unbraked.terminalCurrentA, 0, "Zero brake bound restores coasting reduction");

    // Signed energy and carrier-reaction accounting must hold during braking,
    // not just during startup. The same implicit solver stays monotone.
    omega = highSpeed;
    const double lowSpeed = MotorEquilibrium(p, .3).omegaRadS;
    for (unsigned step = 0; step < 1000; ++step)
    {
      const double previous = omega;
      const auto result = StepMotor(p, previous, .3, .001);
      omega = result.omegaRadS;
      Check(omega <= previous+1e-10 && omega >= lowSpeed-1e-10,
            "Braking converges monotonically without reversing the rotor");
      Check(result.terminalCurrentA >= -p.brakeCurrentLimitA && result.terminalCurrentA <= p.currentLimitA,
            "Signed drive and braking current bounds hold");
      Check(result.effectiveVoltageV >= 0 && result.effectiveVoltageV <= p.voltageV+1e-12,
            "Normal-envelope braking uses feasible bus voltage");
      const double energyChange = .5*p.rotorInertiaKgM2*(omega*omega-previous*previous)/.001;
      Near(p.voltageV*result.batteryCurrentA, result.copperPowerW + result.lossTorqueNm*omega +
           result.propTorqueNm*omega + energyChange + result.numericalDissipationW,
           "Signed braking power closes including returned bus energy", 1e-8);
      for (double spin : {-1., 1.})
      {
        const auto reaction = RotorReaction(p.rotorInertiaKgM2, spin*omega,
          spin*result.alphaRadS2, {0,0,1}, {0,0,0}).Total();
        Near(-spin*result.propTorqueNm+reaction[2], -spin*result.shaftTorqueNm,
             "Braking carrier reaction reverses without double counting", 1e-12);
      }
    }
    Near(omega, lowSpeed, "Braking settles to same positive-duty equilibrium", 1e-8);

    auto stopBrake = p;
    stopBrake.brakeOnStop = true;
    const auto stoppedDrive = StepMotor(stopBrake, highSpeed, 0, .001);
    Check(stoppedDrive.terminalCurrentA < 0, "Separate Brake On Stop option brakes at zero duty");
    const auto defaultStop = StepMotor(p, highSpeed, 0, .001);
    Near(defaultStop.terminalCurrentA, 0, "Default zero-duty stop coasts independently of damping");
    const auto disabled = StepMotor(stopBrake, highSpeed, .7, .001, false);
    Near(disabled.terminalCurrentA, 0, "Disabled drive opens even when Brake On Stop is selected");
    Near(disabled.batteryCurrentA, 0, "Disabled drive cannot regenerate");
    Near(disabled.omegaRadS, defaultStop.omegaRadS, "Drive loss and ordinary zero stop share passive coast");
    stopBrake.brakeCurrentLimitA = 1000;
    const auto shorted = StepMotor(stopBrake, 100, 0, .001);
    Check(shorted.terminalCurrentA < 0, "Zero-duty short has braking current");
    Near(shorted.effectiveVoltageV, 0, "Unclipped zero-duty short has zero applied voltage", 1e-12);
    Near(shorted.batteryCurrentA, 0, "Short braking dissipates energy without bus regeneration", 1e-10);
    Check(shorted.copperPowerW > 0, "Short braking retains copper dissipation");
    Check(StepMotor(p, 10000, .1, .1).omegaRadS < 10000,
          "Overspeed is decelerated by the default brake");
    Check(StepMotor(p, 0, 1, 10).omegaRadS <= p.voltageV/k,
          "Large implicit step remains nonnegative and bounded");

    // In a linear no-load case, halving timestep approaches the exact ODE.
    auto linear = p;
    linear.noLoadCurrentA = 0;
    linear.torqueCoefficientNmPerRadS2 = 0;
    linear.currentLimitA = 1000;
    double coarse = 0, fine = 0;
    for (unsigned i = 0; i < 20; ++i) coarse = StepMotor(linear, coarse, .6, .001).omegaRadS;
    for (unsigned i = 0; i < 40; ++i) fine = StepMotor(linear, fine, .6, .0005).omegaRadS;
    const double exact = .6*p.voltageV/k*(1-std::exp(-.020*k*k/(p.resistanceOhm*p.rotorInertiaKgM2)));
    Check(std::abs(fine-exact) < std::abs(coarse-exact), "Timestep refinement approaches analytic transient");

    bool rejected = false;
    try { StepMotor(p, 0, std::numeric_limits<double>::quiet_NaN(), .001); }
    catch (const std::invalid_argument &) { rejected = true; }
    Check(rejected, "Nonfinite duty rejected");
    auto invalid = p; invalid.rotorInertiaKgM2 = 0; rejected = false;
    try { StepMotor(invalid, 0, .5, .001); }
    catch (const std::invalid_argument &) { rejected = true; }
    Check(rejected, "Missing rotor inertia rejected");
    invalid = p; invalid.noLoadVoltageV = invalid.resistanceOhm*invalid.noLoadCurrentA;
    rejected = false;
    try { StepMotor(invalid, 0, .5, .001); }
    catch (const std::invalid_argument &) { rejected = true; }
    Check(rejected, "Impossible no-load reference voltage rejected");
    invalid = p; invalid.brakeCurrentLimitA = -1; rejected = false;
    try { StepMotor(invalid, 0, .5, .001); }
    catch (const std::invalid_argument &) { rejected = true; }
    Check(rejected, "Negative brake-current bound rejected");
    invalid.brakeCurrentLimitA = std::numeric_limits<double>::infinity(); rejected = false;
    try { StepMotor(invalid, 0, .5, .001); }
    catch (const std::invalid_argument &) { rejected = true; }
    Check(rejected, "Nonfinite brake-current bound rejected");
    std::cout << "Passed " << checks << " motor torque / energy / integration checks\n";
    return 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
