// Focused algebra and native-wire regression checks; no simulator dependency.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "TricopterMath.hh"
#include "sitl_gyro.h"
#include <iostream>
#include <deque>
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
void PutFloat(uint8_t *p, float value)
{
  uint32_t bits; std::memcpy(&bits, &value, sizeof(bits)); tricopter::Write32(p, bits);
}
double GetDouble(const uint8_t *p)
{
  uint64_t bits = 0;
  for (unsigned i = 0; i < 8; ++i) bits |= static_cast<uint64_t>(p[i]) << (8*i);
  double value; std::memcpy(&value, &bits, sizeof(value)); return value;
}
}

int main()
{
  try
  {
    using namespace tricopter;
    Calibration c; c.Validate();
    for (unsigned i = 0; i < 3; ++i)
    {
      Near(c.TargetRadians(c.pwm[i]), c.angleDeg[i]*kPi/180, "PWM knot");
      Check(c.PositionAdc(c.angleDeg[i]*kPi/180) == c.adc[i], "ADC knot");
    }
    Near(c.TargetRadians(0), -47.8*kPi/180, "Lower PWM clamp");
    Near(c.TargetRadians(2500), 21.6*kPi/180, "Upper PWM clamp");
    Near(c.TargetRadians((1100+1509)/2.0), -23.9*kPi/180, "Lower interpolation");
    Near(c.TargetRadians((1509+1700)/2.0), 10.8*kPi/180, "Upper interpolation");
    Check(c.PositionAdc(-23.9*kPi/180) == 1713, "Measured actual ADC interpolation");
    Check(c.PositionAdc(-2) == 1936 && c.PositionAdc(2) == 1281, "ADC endpoint clamp");
    auto bad = c; bad.adc[1] = bad.adc[0];
    bool rejected = false;
    try { bad.Validate(); } catch (const std::invalid_argument &) { rejected = true; }
    Check(rejected, "Invalid calibration rejected");

    std::array<uint8_t, kRawPwmBytes> raw {};
    Write16(raw.data(), 3); raw[2] = 0xa5; raw[3] = 0x5a; // native padding is ignored
    for (unsigned i = 0; i < 4; ++i) PutFloat(raw.data()+4+4*i, 1100+100*i);
    Actuators a;
    Check(DecodeRawPwm(raw.data(), raw.size(), a), "Stock raw ABI accepted");
    Near(a.motorPwm[0], 1100, "Tail0 order"); Near(a.motorPwm[1], 1200, "Right1 order");
    Near(a.motorPwm[2], 1300, "Left2 order"); Near(a.servoPwm, 1400, "Physical servo0 appended");
    Check(!DecodeRawPwm(raw.data(), 67, a), "Truncated datagram rejected");
    Write16(raw.data(), 4); Check(!DecodeRawPwm(raw.data(), raw.size(), a), "Quad output rejected");
    Write16(raw.data(), 3); PutFloat(raw.data()+4, std::numeric_limits<float>::quiet_NaN());
    Check(!DecodeRawPwm(raw.data(), raw.size(), a), "NaN PWM rejected");
    Near(a.motorPwm[0], 1100, "Rejected packet does not alter prior actuator state");
    PutFloat(raw.data()+4, 1100); PutFloat(raw.data()+16, 0);
    Check(DecodeRawPwm(raw.data(), raw.size(), a) && a.servoPwm == 0, "Suppressed servo supported");
    PutFloat(raw.data()+16, 100); Check(!DecodeRawPwm(raw.data(), raw.size(), a), "Malformed servo rejected");

    // Drive the actual bounded receive-drain helper using an in-memory UDP
    // queue. A backlog larger than one tick must never become fresh actuation.
    ActuatorGate gate;
    std::deque<std::array<uint8_t, kRawPwmBytes>> queue;
    unsigned acceptedPackets = 0;
    const auto drain = [&]()
    {
      DrainActuatorQueue(gate,
        [&queue](uint8_t *destination, std::size_t) -> int
        {
          if (queue.empty()) return -1;
          std::memcpy(destination, queue.front().data(), queue.front().size());
          queue.pop_front(); return kRawPwmBytes;
        },
        [&acceptedPackets](const Actuators &) { ++acceptedPackets; });
    };
    PutFloat(raw.data()+16, 1509);
    gate.StateSent(); Check(!gate.IsAccepting(), "FDM alone cannot bypass initial queue drain");
    drain(); Check(!gate.IsAccepting(), "Empty queue alone cannot enable actuation");
    gate.StateSent(); Check(gate.IsAccepting() && !gate.CanDrive(), "New FDM still requires new actuator receipt");
    queue.push_back(raw); drain();
    Check(acceptedPackets == 1 && gate.CanDrive(), "Post-state packet accepted");
    gate.Invalidate(); // Pause boundary: retain physical motor state outside gate.
    Check(!gate.CanDrive(), "Pause invalidates previous actuator liveness");
    for (unsigned i = 0; i < 300; ++i) queue.push_back(raw);
    drain(); gate.StateSent();
    Check(queue.size() == 172 && !gate.IsAccepting(), "Pause backlog exceeding128 stays closed");
    drain(); gate.StateSent();
    Check(queue.size() == 44 && !gate.IsAccepting(), "Remaining stale backlog cannot be enabled by FDM");
    drain(); Check(queue.empty() && !gate.IsAccepting(), "Final stale packets discarded before state send");
    Check(acceptedPackets == 1, "No queued pre-pause packet accepted");
    // A packet arriving after the early drain, but before the new state, is
    // discarded by the second PostUpdate boundary drain too.
    queue.push_back(raw); drain();
    Check(acceptedPackets == 1 && !gate.IsAccepting(), "Pre-FDM arrival after early drain discarded");
    gate.StateSent(); Check(!gate.CanDrive(), "Resume does not reuse previous accepted command");
    queue.push_back(raw); drain();
    Check(acceptedPackets == 2 && gate.CanDrive(), "Fresh post-resume packet restores drive");
    gate.Invalidate(); queue.push_back(raw); drain(); // World reset boundary.
    Check(acceptedPackets == 2 && !gate.CanDrive(), "Reset queue discarded and liveness cleared");
    gate.StateSent(); Check(!gate.CanDrive(), "Reset new FDM cannot restore stale actuator state");
    queue.push_back(raw); drain();
    Check(acceptedPackets == 3 && gate.CanDrive(), "Fresh post-reset packet accepted");

    FdmPacer pacer;
    Check(pacer.CanSend(0, 0), "Initial FDM can be sent");
    pacer.Sent(0, 0);
    Check(!pacer.CanSend(.001, .001), "No second normal FDM before raw response");
    Check(!pacer.CanSend(.029, .029), "Startup retry waits30ms");
    Check(!pacer.CanSend(.030, .010), "Retry cannot burst tiny simulation deltas");
    Check(!pacer.CanSend(.010, .030), "Retry cannot burst tiny wall deltas");
    Check(pacer.CanSend(.030, .030), "No-raw startup retry allowed at30ms");
    pacer.Sent(.030, .030); pacer.RawReply(.031);
    Check(!pacer.CanSend(.0315, .032), "Normal FDM waits1ms after raw ACK");
    Check(pacer.CanSend(.032, .032), "ACK permits paced next FDM");
    pacer.Sent(.032, .032);
    Check(!pacer.CanSend(.033, .033), "One normal FDM remains outstanding");
    pacer.RawReply(.040); pacer.RawReply(.041); // Batched late replies.
    Check(!pacer.CanSend(.0415, .042), "Latest batched ACK controls spacing");
    Check(pacer.CanSend(.042, .042), "Batched replies do not permit immediate burst");
    pacer.Sent(.042, .042);
    Check(!pacer.CanSend(.100, .043), "Long wall stall still requires safe retry simulation delta");
    Check(pacer.CanSend(.100, .100), "Long stall may send latest state after safe retry interval");
    Check(!pacer.CanSend(.041, .100), "Backward wall time rejected");
    Check(!pacer.CanSend(.100, .042), "Repeated simulation time rejected");
    pacer.Reset(); Check(pacer.CanSend(.101, 0), "World reset clears prior timestamp epoch");
    pacer.Sent(.101, 0);
    Check(!pacer.CanSend(.102, .001), "Reset again requires a new raw reply");
    Check(!pacer.CanSend(std::numeric_limits<double>::quiet_NaN(), .03), "Nonfinite pacer input rejected");

    const auto adc = EncodeAdc(12.5, 0x12345678, 123, 1490);
    Check(adc.size() == 32 && Read32(adc.data()) == 0x31414454, "ADC magic and size");
    Check(Read16(adc.data()+4) == 1 && Read16(adc.data()+6) == 32, "ADC version / encoded length");
    Near(GetDouble(adc.data()+8), 12.5, "ADC simulation timestamp");
    Check(Read32(adc.data()+16) == 0x12345678 && Read32(adc.data()+20) == 123, "ADC epoch / sequence");
    Check(Read16(adc.data()+24) == 1490 && Read16(adc.data()+26) == 0, "PB0 and PC0=zero");
    Check(Read32(adc.data()+28) == 0, "ADC reserved zero");
    rejected = false;
    try { (void)EncodeAdc(1, 2, 3, 4096); } catch (const std::invalid_argument &) { rejected = true; }
    Check(rejected, "Non12bit ADC rejected");
    std::array<double, 18> fdm {};
    for (unsigned i = 0; i < fdm.size(); ++i) fdm[i] = i+0.25;
    const auto encoded = EncodeFdm(fdm);
    Check(encoded.size() == 144, "Legacy FDM exactly144bytes");
    for (unsigned i = 0; i < fdm.size(); ++i) Near(GetDouble(encoded.data()+8*i), fdm[i], "FDM wire ordering");

    // Exercise the actual native SITL gyro conversion, not a mirrored test stub.
    const auto gyro = BodyFluToSensorFrd({.3, -.4, .5});
    double roll, pitch, yaw;
    sitlGyroBodyFromSim(gyro.data(), true, &roll, &pitch, &yaw);
    Near(roll, .3, "Native gyro roll = FLU+X");
    Near(pitch, -.4, "Native gyro pitch = FLU+Y nose-down");
    Near(yaw, -.5, "Native gyro yaw = FLU-Z clockwise");
    const auto packetAcc = FdmAccelerationFromSpecificFlu({-2.3, 1.1, 9.1});
    Near(-packetAcc[0], -2.3, "Native acc X preserved after unconditional negate");
    Near(-packetAcc[1], 1.1, "Native acc Y preserved after unconditional negate");
    Near(-packetAcc[2], 9.1, "Native acc Z preserved after unconditional negate");
    const auto level = FdmAccelerationFromSpecificFlu({0, 0, kGravity});
    Near(level[2], -kGravity, "Level packet -1g becomes native +1g");
    const auto q = FdmQuaternion({.5, .1, .2, .3});
    Near(q[0], .5, "Quaternion W"); Near(q[1], .1, "Quaternion X");
    Near(-q[2], .2, "Native undo conjugation Y"); Near(-q[3], .3, "Native undo conjugation Z");

    const double angle = 20*kPi/180;
    const auto axis = RearAxis(angle);
    Near(axis[0], 0, "Rear force axis X");
    Near(axis[1], std::sin(angle), "Rear force follows actual joint angle Y");
    Near(axis[2], std::cos(angle), "Rear force follows actual joint angle Z");
    Near(axis[1]*axis[1]+axis[2]*axis[2], 1, "Rear force unit direction");
    Check(-.32*axis[1] < 0, "Positive joint angle makes clockwise yaw torque");
    Check(RearAxis(-angle)[1] < 0, "Reversed joint angle reverses lateral thrust");
    for (const double spin : {-1.0, 1.0})
    {
      const double thrust = 4, reaction = -spin*.02*thrust;
      Near(reaction, -spin*.08, "Reaction sign opposes rotor spin");
      Near(reaction*axis[2], -spin*.08*std::cos(angle), "Tilted reaction Z projection");
      Near(reaction*axis[1], -spin*.08*std::sin(angle), "Tilted reaction Y projection");
    }
    double target = 0;
    for (unsigned i = 0; i < 100; ++i)
    {
      const double next = ServoTargetStep(target, angle, .001, .02, 1.0);
      Check(next >= target && next-target <= .001+1e-12, "Servo target lag / slew bound");
      target = next;
    }
    Near(target, .1, "Servo rate bound cumulative");
    Near(LagStep(0, 1, .03, .03), 1-std::exp(-1), "Motor exact first-order response");
    Near(LagStep(.5, 1, 0, .03), .5, "Zero dt no dynamics advance");
    std::cout << "Passed " << checks << " tricopter math/protocol checks\n";
    return 0;
  }
  catch (const std::exception &e)
  {
    std::cerr << "Failure after " << checks << " checks: " << e.what() << '\n';
    return 1;
  }
}
