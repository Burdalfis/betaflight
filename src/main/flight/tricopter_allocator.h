/* Offline geometric tricopter wrench allocation; no actuator/control I/O. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#define TRICOPTER_ALLOCATOR_MOTORS 3U
#define TRICOPTER_ALLOCATOR_WRENCH_AXES 4U
#define TRICOPTER_ALLOCATOR_MAX_ITERATIONS 4096U

typedef enum tricopterAllocatorSource_e {
    TRICOPTER_ALLOCATOR_SOURCE_UNSPECIFIED = 0,
    TRICOPTER_ALLOCATOR_SOURCE_SYNTHETIC_PLACEHOLDER = 1,
} tricopterAllocatorSource_e;

// Generic virtual-tail basis, NOT a local [delta-motors,delta-servo] Jacobian.
// Columns multiply [frontRightEffort,frontLeftEffort,tailEffort*cos(direction),
// tailEffort*sin(direction)]. Matrix/response units are explicitly the user's
// characterized proxy units. Direction is mathematical until calibrated;
// this API requires no geometry, CG, kT, physical thrust, or rotor angle.
typedef struct tricopterAllocatorVirtualParams_s {
    float effectivenessMatrix[4][4]; // Each finite coefficient magnitude <=1e6.
    float maxEffort[3]; // Logical tail/frontRight/frontLeft, 0<max<=1e6.
    float tailDirectionMinRad;
    float tailDirectionMaxRad;
    float responseScale[4]; // Positive proxy response units, [1e-6,1e6].
    float lowTailEffort;
    float projectedGradientTolerance;
    float maxConditionNumber;
    uint16_t maxIterations;
    tricopterAllocatorSource_e source;
} tricopterAllocatorVirtualParams_t;

typedef struct tricopterAllocatorVirtualRequest_s {
    float response[4]; // Each finite magnitude <=1e8, same units as matrix rows.
} tricopterAllocatorVirtualRequest_t;

typedef struct tricopterAllocatorVirtualOutput_s {
    float requestedResponse[4];
    float achievedResponse[4];
    float residualResponse[4]; // achieved minus requested.
    float effort[3];
    float controls[4]; // [frontRight,frontLeft,tailVerticalProxy,tailRightProxy].
    float tailDirectionRad;
    float cost;
    float projectedGradientNorm;
    float conditionNumber;
    uint32_t status;
    uint32_t constraints;
    uint16_t iterations;
    bool valid;
    bool converged;
    bool exact; // Unconstrained inverse feasible before float32 publication.
    bool tailDirectionDefined;
    bool tailEffortLow;
    bool calibrated; // Always false.
    bool forFlight; // Always false.
} tricopterAllocatorVirtualOutput_t;

typedef struct tricopterAllocatorParams_s {
    // Body x forward, y left, z up; all positions relative to CG, meters.
    // Logical order tail0, front-right1, front-left2. Front thrust is +z.
    float motorPositionM[3][3];
    float maxThrustN[3]; // 0 < max <= 1e6; no positive minimum thrust.
    float kT[3]; // Explicit N/(rad/s)^2, in [1e-12,1e3].
    float tailAngleMinRad; // -pi/2 < min <= 0 <= max < pi/2.
    float tailAngleMaxRad;
    float wrenchScale[4]; // Positive [N,Nm,Nm,Nm], each [1e-6,1e6].
    float lowTailThrustN; // Diagnostic threshold only, 0..tail maximum.
    float projectedGradientTolerance; // Norm of gradient mapping, 1/N.
    float maxConditionNumber; // Weighted matrix infinity-norm guard, [1,1e8].
    uint16_t maxIterations; // 1..4096, fixed upper bound; no hidden retries.
    tricopterAllocatorSource_e source;
} tricopterAllocatorParams_t;

typedef struct tricopterAllocatorRequest_s {
    float wrench[4]; // [Fz N, Mx Nm, My Nm, Mz Nm], each magnitude <=1e8.
    // Exogenous known yaw only. No nonlinear torque model evaluation here.
    float reactionYawNm;
    bool useReactionYaw;
    bool reactionYawValid;
} tricopterAllocatorRequest_t;

typedef enum tricopterAllocatorStatus_e {
    TRICOPTER_ALLOCATOR_OK = 0,
    TRICOPTER_ALLOCATOR_INVALID_CONFIG = 1U << 0,
    TRICOPTER_ALLOCATOR_INVALID_INPUT = 1U << 1,
    TRICOPTER_ALLOCATOR_DEGENERATE_GEOMETRY = 1U << 2,
    TRICOPTER_ALLOCATOR_ILL_CONDITIONED = 1U << 3,
    TRICOPTER_ALLOCATOR_REACTION_UNAVAILABLE = 1U << 4,
    TRICOPTER_ALLOCATOR_FORWARD_BOUNDS = 1U << 5,
    TRICOPTER_ALLOCATOR_NOT_CONVERGED = 1U << 6,
} tricopterAllocatorStatus_e;

typedef enum tricopterAllocatorConstraint_e {
    TRICOPTER_ALLOCATOR_TAIL_ZERO = 1U << 0,
    TRICOPTER_ALLOCATOR_TAIL_MAX = 1U << 1,
    TRICOPTER_ALLOCATOR_FRONT_RIGHT_ZERO = 1U << 2,
    TRICOPTER_ALLOCATOR_FRONT_RIGHT_MAX = 1U << 3,
    TRICOPTER_ALLOCATOR_FRONT_LEFT_ZERO = 1U << 4,
    TRICOPTER_ALLOCATOR_FRONT_LEFT_MAX = 1U << 5,
    TRICOPTER_ALLOCATOR_ANGLE_MIN = 1U << 6,
    TRICOPTER_ALLOCATOR_ANGLE_MAX = 1U << 7,
} tricopterAllocatorConstraint_e;

typedef struct tricopterAllocatorOutput_s {
    float requestedWrench[4];
    float achievedWrench[4]; // Includes exogenous reaction yaw if enabled.
    float residualWrench[4]; // achieved minus requested.
    float forceN[3]; // Full body [Fx,Fy,Fz]; Fy=-tailRightwardN.
    float thrustN[3]; // Logical tail/front-right/front-left.
    float omegaRadS[3];
    float mechanicalRpm[3];
    float controlsN[4]; // [frontRightT,frontLeftT,tailVertical,tailRightward].
    float tailAngleRad; // atan2(h,v); exactly zero at a zero tail vector.
    float cost; // 0.5 * sum((residual/scale)^2).
    float projectedGradientNorm; // ||x-project(x-step*gradient)||/step.
    float conditionNumber;
    uint32_t status;
    uint32_t constraints;
    uint16_t iterations;
    bool valid; // Finite feasible controls; does NOT imply convergence.
    bool converged;
    bool exact; // Inverse feasible before publication, within 1e-10 relative N.
    bool tailAngleDefined; // True for nonzero tail vector, even below threshold.
    bool tailThrustLow; // Diagnostic only; no control/angle is changed by it.
    bool calibrated; // Always false.
    bool forFlight; // Always false.
} tricopterAllocatorOutput_t;

// Binary32 inputs/binary64 arithmetic. Positions must have magnitude <=1000m;
// weighted 4x4 allocation matrix must be invertible and satisfy the explicit
// condition guard. Float guards remain effective with -ffast-math.
bool tricopterAllocatorValidateParams(const tricopterAllocatorParams_t *params);

// Exact inverse when feasible; otherwise projected gradient on a convex
// front-interval x front-interval x tail circular-sector set. Objective is
// one-half weighted squared wrench residual; step=1/trace(A'WA). Bounded
// iterations expose nonconvergence rather than declaring exact optimality.
// Invalid inputs/config clear every numeric output. Unconverged valid output
// retains feasible controls with NOT_CONVERGED status and converged=false.
tricopterAllocatorOutput_t tricopterAllocatorAllocate(const tricopterAllocatorParams_t *params,
    const tricopterAllocatorRequest_t *request);

// Independent geometric forward evaluation for actual/predicted fixed angle.
// Thrusts and angle must lie within configured bounds; no allocation or angle
// inference from PWM/ADC. requestedWrench is set equal to achievedWrench,
// residual/cost/iterations zero; exact/converged mean forward evaluation only.
tricopterAllocatorOutput_t tricopterAllocatorForward(const tricopterAllocatorParams_t *params,
    const float thrustN[3], float tailAngleRad, float reactionYawNm,
    bool useReactionYaw, bool reactionYawValid);

// Shared convex kernel using only virtual effort/response units. Same bounded
// algorithm and status/constraint flags as SI adapter; no physical conversion.
bool tricopterAllocatorVirtualValidateParams(const tricopterAllocatorVirtualParams_t *params);
tricopterAllocatorVirtualOutput_t tricopterAllocatorVirtualAllocate(const tricopterAllocatorVirtualParams_t *params,
    const tricopterAllocatorVirtualRequest_t *request);
tricopterAllocatorVirtualOutput_t tricopterAllocatorVirtualForward(const tricopterAllocatorVirtualParams_t *params,
    const float effort[3], float tailDirectionRad);
