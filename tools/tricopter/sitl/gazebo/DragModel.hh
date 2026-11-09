// Lumped body-axis airframe drag; independent of Gazebo.
// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "TricopterMath.hh"

namespace tricopter
{
struct AirframeDragParameters
{
  double densityKgM3 {1.225};
  // Engineering placeholders, not measured airframe Cd or projected areas.
  Vector cdaBodyM2 {.010, .012, .025};

  void Validate() const
  {
    if (!std::isfinite(densityKgM3) || densityKgM3 <= 0)
      throw std::invalid_argument("Air density must be positive and finite");
    for (double value : cdaBodyM2)
      if (!std::isfinite(value) || value < 0)
        throw std::invalid_argument("Drag CdA must be nonnegative and finite");
  }
};

struct AirframeDragResult
{
  Vector velocityBodyMPS {}, forceBodyN {}, forceWorldN {};
  double powerW {0};
};

// Each element is one body FLU unit axis expressed in world coordinates.
// With an orthonormal basis, F.v = -rho/2 * sum(CdA_i * |v_i|^3) <= 0.
// The caller supplies air-relative velocity, not ground speed or rotor inflow.
inline AirframeDragResult QuadraticAirframeDrag(
    const AirframeDragParameters &parameters, const Vector &velocityWorldMPS,
    const std::array<Vector, 3> &bodyAxesWorld)
{
  AirframeDragResult result;
  for (unsigned i = 0; i < 3; ++i)
  {
    for (unsigned j = 0; j < 3; ++j)
      result.velocityBodyMPS[i] += bodyAxesWorld[i][j]*velocityWorldMPS[j];
    const double velocity = result.velocityBodyMPS[i];
    result.forceBodyN[i] = -.5*parameters.densityKgM3*parameters.cdaBodyM2[i]*
                          std::abs(velocity)*velocity;
    result.powerW += result.forceBodyN[i]*velocity;
    for (unsigned j = 0; j < 3; ++j)
      result.forceWorldN[j] += bodyAxesWorld[i][j]*result.forceBodyN[i];
  }
  return result;
}
}
