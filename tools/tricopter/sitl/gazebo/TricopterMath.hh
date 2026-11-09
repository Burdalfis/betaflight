// Shared simulator math / wire encoding, independent of Gazebo.
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>

namespace tricopter
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kGravity = 9.80665;
constexpr std::size_t kRawPwmBytes = 68;
constexpr std::size_t kFdmBytes = 144;
constexpr std::size_t kAdcBytes = 32;
using Vector = std::array<double, 3>;

inline double Clamp(double x, double lo, double hi)
{
  return std::max(lo, std::min(hi, x));
}

inline bool Monotonic(const std::array<double, 3> &x)
{
  return (x[0] < x[1] && x[1] < x[2]) ||
    (x[0] > x[1] && x[1] > x[2]);
}

inline double MapThree(const std::array<double, 3> &x,
                       const std::array<double, 3> &y, double query)
{
  if (!std::isfinite(query) || !Monotonic(x))
    throw std::invalid_argument("Invalid three-point map");
  for (unsigned i = 0; i < 3; ++i)
    if (!std::isfinite(x[i]) || !std::isfinite(y[i]))
      throw std::invalid_argument("Nonfinite three-point map");
  query = Clamp(query, std::min(x.front(), x.back()),
                std::max(x.front(), x.back()));
  const unsigned i = ((x[1] > x[0]) ? query <= x[1] : query >= x[1]) ? 0 : 1;
  return y[i] + (query - x[i]) / (x[i+1] - x[i]) * (y[i+1] - y[i]);
}

struct Calibration
{
  std::array<double, 3> pwm {1100, 1509, 1700};
  std::array<double, 3> angleDeg {-47.8, 0, 21.6};
  std::array<double, 3> adc {1936, 1490, 1281};

  void Validate() const
  {
    if (!(pwm[0] < pwm[1] && pwm[1] < pwm[2]) ||
        !(angleDeg[0] < 0 && angleDeg[1] == 0 && angleDeg[2] > 0) ||
        !Monotonic(adc))
      throw std::invalid_argument("Invalid PWM/angle/ADC calibration");
    for (unsigned i = 0; i < 3; ++i)
      if (!std::isfinite(pwm[i]) || pwm[i] < 500 || pwm[i] > 2500 ||
          !std::isfinite(angleDeg[i]) || std::abs(angleDeg[i]) >= 85 ||
          !std::isfinite(adc[i]) || adc[i] < 0 || adc[i] > 4095)
        throw std::invalid_argument("Calibration point out of range");
  }

  double TargetRadians(double pwmUs) const
  {
    return MapThree(pwm, angleDeg, pwmUs) * kPi / 180;
  }

  uint16_t PositionAdc(double jointRadians) const
  {
    return static_cast<uint16_t>(std::lround(Clamp(
      MapThree(angleDeg, adc, jointRadians * 180 / kPi), 0, 4095)));
  }
};

inline uint16_t Read16(const uint8_t *p)
{
  return static_cast<uint16_t>(p[0] | static_cast<uint16_t>(p[1]) << 8);
}
inline uint32_t Read32(const uint8_t *p)
{
  return static_cast<uint32_t>(p[0]) | static_cast<uint32_t>(p[1]) << 8 |
    static_cast<uint32_t>(p[2]) << 16 | static_cast<uint32_t>(p[3]) << 24;
}
inline void Write16(uint8_t *p, uint16_t n)
{
  for (unsigned i = 0; i < 2; ++i) p[i] = (n >> (8*i)) & 255;
}
inline void Write32(uint8_t *p, uint32_t n)
{
  for (unsigned i = 0; i < 4; ++i) p[i] = (n >> (8*i)) & 255;
}
inline void WriteDouble(uint8_t *p, double value)
{
  static_assert(sizeof(double) == 8 && sizeof(float) == 4, "IEEE wire sizes");
  uint64_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  for (unsigned i = 0; i < 8; ++i) p[i] = (bits >> (8*i)) & 255;
}

struct Actuators
{
  std::array<double, 3> motorPwm {1000, 1000, 1000};
  double servoPwm {1509};
};

// Existing Betaflight stock ABI: little-endian uint16 motorCount, two bytes
// native padding, sixteen IEEE float outputs. Physical servo0 follows motor2.
inline bool DecodeRawPwm(const uint8_t *bytes, std::size_t size, Actuators &out)
{
  if (size != kRawPwmBytes || Read16(bytes) != 3) return false;
  std::array<double, 4> values;
  for (unsigned i = 0; i < 4; ++i)
  {
    const uint32_t bits = Read32(bytes + 4 + i*4);
    float f;
    std::memcpy(&f, &bits, sizeof(f));
    values[i] = f;
    if (!std::isfinite(values[i]) || values[i] < 0 || values[i] > 2500)
      return false;
  }
  // A zero servo pulse is the existing disarmed-suppression convention.
  if (values[3] != 0 && (values[3] < 500 || values[3] > 2500)) return false;
  for (unsigned i = 0; i < 3; ++i) out.motorPwm[i] = values[i];
  out.servoPwm = values[3];
  return true;
}

// The stock raw PWM ABI has no timestamp. Across pause/reset boundaries, drain
// the old receive queue completely, send a new FDM state, then require a newly
// received actuator packet. A bounded drain that has not reached EAGAIN must
// remain closed rather than treating its leftover backlog as fresh control.
class ActuatorGate
{
public:
  void Invalidate() { stage = Stage::Discarding; accepted = false; }
  void QueueDrained()
  {
    if (stage == Stage::Discarding) stage = Stage::AwaitingState;
  }
  void StateSent()
  {
    if (stage == Stage::AwaitingState) stage = Stage::Active;
  }
  bool IsAccepting() const { return stage == Stage::Active; }
  bool CanDrive() const { return IsAccepting() && accepted; }
  void PacketAccepted() { if (IsAccepting()) accepted = true; }
private:
  enum class Stage { Discarding, AwaitingState, Active };
  Stage stage {Stage::Discarding};
  bool accepted {false};
};

// Native SITL estimates its clock rate from consecutive FDM receive intervals.
// Streaming faster than it consumes can queue/burst packets and accelerate that
// clock enough to falsely time out live RC. Keep one normal FDM outstanding and
// leave >=1ms after its raw-PWM response. Lost/startup responses permit retries,
// spaced >=30ms in BOTH clocks: native SITL skips rate estimation for >=20ms
// simulation deltas, so a late queued retry pair cannot create a huge rate.
class FdmPacer
{
public:
  void Reset() { sent = outstanding = acknowledged = false; }
  bool CanSend(double wallTime, double simulationTime) const
  {
    if (!std::isfinite(wallTime) || !std::isfinite(simulationTime) || simulationTime < 0)
      return false;
    if (!sent) return true;
    if (simulationTime <= lastSimulationTime || wallTime < lastWallTime) return false;
    if (outstanding)
      return wallTime-lastWallTime >= kRetrySeconds-1e-9 &&
             simulationTime-lastSimulationTime >= kRetrySeconds-1e-9;
    return wallTime-lastWallTime >= kMinimumSeconds-1e-9 &&
           (!acknowledged || wallTime-lastAckWallTime >= kMinimumSeconds-1e-9);
  }
  void Sent(double wallTime, double simulationTime)
  {
    sent = outstanding = true;
    lastWallTime = wallTime; lastSimulationTime = simulationTime;
  }
  void RawReply(double wallTime)
  {
    if (!sent) return;
    outstanding = false; acknowledged = true; lastAckWallTime = wallTime;
  }
private:
  static constexpr double kMinimumSeconds = .001;
  static constexpr double kRetrySeconds = .030;
  bool sent {false}, outstanding {false}, acknowledged {false};
  double lastWallTime {0}, lastSimulationTime {0}, lastAckWallTime {0};
};

// Receiver returns -1 only for an empty nonblocking queue, -2 for other errors.
// This same bounded drain is used by the actual plugin and the queue regression.
template<class Receive, class Apply>
void DrainActuatorQueue(ActuatorGate &gate, Receive receive, Apply apply,
                         unsigned maxDatagrams = 128)
{
  std::array<uint8_t, 256> bytes {};
  for (unsigned i = 0; i < maxDatagrams; ++i)
  {
    const auto n = receive(bytes.data(), bytes.size());
    if (n == -1) { gate.QueueDrained(); return; }
    if (n < 0) { gate.Invalidate(); return; }
    Actuators candidate;
    if (gate.IsAccepting() && DecodeRawPwm(bytes.data(), static_cast<std::size_t>(n), candidate))
    {
      gate.PacketAccepted(); apply(candidate);
    }
  }
  if (!gate.IsAccepting()) gate.Invalidate(); // Backlog may remain after the bound.
}

inline std::array<uint8_t, kFdmBytes> EncodeFdm(const std::array<double, 18> &fields)
{
  std::array<uint8_t, kFdmBytes> bytes {};
  for (unsigned i = 0; i < fields.size(); ++i)
  {
    if (!std::isfinite(fields[i])) throw std::invalid_argument("Nonfinite FDM");
    WriteDouble(bytes.data()+8*i, fields[i]);
  }
  return bytes;
}

inline std::array<uint8_t, kAdcBytes> EncodeAdc(double time, uint32_t epoch,
                                             uint32_t sequence, uint16_t pb0)
{
  if (!std::isfinite(time) || time < 0 || pb0 > 4095)
    throw std::invalid_argument("Invalid ADC sample");
  std::array<uint8_t, kAdcBytes> bytes {};
  Write32(bytes.data(), 0x31414454); // "TDA1"
  Write16(bytes.data()+4, 1);
  Write16(bytes.data()+6, kAdcBytes);
  WriteDouble(bytes.data()+8, time);
  Write32(bytes.data()+16, epoch);
  Write32(bytes.data()+20, sequence);
  Write16(bytes.data()+24, pb0);
  Write16(bytes.data()+26, 0); // PC0 is exactly zero in this plant.
  return bytes;
}

// Axis of the rear rotor for the supplied joint axis -X in the body FLU frame.
// Positive angle gives leftward thrust (+Y), hence clockwise Betaflight yaw.
inline Vector RearAxis(double angle)
{
  return {0, std::sin(angle), std::cos(angle)};
}

struct RotorInertialTorque
{
  Vector spinup {};
  Vector precession {};

  Vector Total() const
  {
    return {spinup[0]+precession[0], spinup[1]+precession[1],
            spinup[2]+precession[2]};
  }
};

// The physics link includes the rotor's locked mass and inertia. Relative spin
// contributes additional angular momentum H = J * signedSpeed * axis. Apply
// -dH/dt to the carrier, including reorientation by its actual angular velocity.
// Vectors share a frame; axis is unit length. Speeds use rad/s, acceleration
// rad/s^2, J kg*m^2, and output torques N*m. A stopped rotor has no precession.
inline RotorInertialTorque RotorReaction(double polarInertia, double signedSpeed,
    double signedAcceleration, const Vector &axis, const Vector &angularVelocity)
{
  const Vector axisRate {
    angularVelocity[1]*axis[2]-angularVelocity[2]*axis[1],
    angularVelocity[2]*axis[0]-angularVelocity[0]*axis[2],
    angularVelocity[0]*axis[1]-angularVelocity[1]*axis[0]};
  RotorInertialTorque result;
  for (unsigned i = 0; i < 3; ++i)
  {
    result.spinup[i] = -polarInertia*signedAcceleration*axis[i];
    result.precession[i] = -polarInertia*signedSpeed*axisRate[i];
  }
  return result;
}

inline Vector BodyFluToSensorFrd(const Vector &v)
{
  return {v[0], -v[1], -v[2]};
}

inline Vector FdmAccelerationFromSpecificFlu(const Vector &v)
{
  // sitl.c negates ALL three packet axes before virtualAccSet. This is a
  // legacy wire convention, not the FRD gyro transform above.
  return {-v[0], -v[1], -v[2]};
}

inline std::array<double, 4> FdmQuaternion(const std::array<double, 4> &fluToEnu)
{
  return {fluToEnu[0], fluToEnu[1], -fluToEnu[2], -fluToEnu[3]};
}

inline double LagStep(double state, double target, double dt, double tau)
{
  if (dt <= 0) return state;
  return target + (state-target) * std::exp(-dt/tau);
}

inline double ServoTargetStep(double state, double target, double dt,
                               double tau, double rateRadPerSec)
{
  const double filtered = LagStep(state, target, dt, tau);
  return state + Clamp(filtered-state, -rateRadPerSec*dt, rateRadPerSec*dt);
}
}
