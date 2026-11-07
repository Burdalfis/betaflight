/* Shared Python/MATLAB transport for SYNTHETIC offline simulation only.
 * All matrices are contiguous ROW MAJOR. Commands from row i affect the plant
 * AFTER row i, never its recorded truth. No hardware/firmware I/O is linked.
 */
#pragma once
#include <stddef.h>
#include <stdint.h>

#define SIMULATION_CONTROLLER_PARAMETERS 75U
#define SIMULATION_PLANT_PARAMETERS 74U
#define SIMULATION_NUMERIC_COLUMNS 85U
#define SIMULATION_INTEGER_COLUMNS 27U
#define SIMULATION_FLAG_COLUMNS 47U
#define SIMULATION_PACKET_CAPACITY 64U

/* controller double[75]:
 * 0..28 allocator virtual_parameters layout from allocator_native.py:
 * B[16],maxEffort[3],minDirection,maxDirection,responseScale[4],lowEffort,
 * gradientTolerance,maxCondition,maxIterations.
 * 29..41 servo: PWM[3],counts[3],play,delayUs,tauInc,tauDec,rateInc,rateDec,maxGapUs.
 * 42..48 observer: trackingTau,decayTau,maxRpm,maxAcceleration,staleUs,horizonUs,gapUs.
 * 49..61 reaction: kQ[3],J[3],sign[3],maxRpm,maxAcceleration,maxTorque,maxAgeUs.
 * 62..64 effortPerOmegaSquared[3]; 65/66 countsAtMin/MaxDirection;
 * 67 yawResponsePerNm;68 maxAbsReactionResponse;69 minSupportCosine;
 * 70 effectivenessExcludesReaction (must1);71 initialCounts;72 initialPWM;
 * 73 initialApproach(-1,0,+1);74 initialCorrectedPositionValid(must1).
 * All coefficient-source enums are SYNTHETIC_PLACEHOLDER, never calibrated.
 */
/* plant double[74]: B[16] (0..15),servo[13] (16..28),motorTauInc[3](29),
 * motorTauDec[3](32),motorSlewInc[3](35),motorSlewDec[3](38),
 * effortPerOmegaSquared[3](41),reaction[13](44),countsAtMin/MaxDirection(57/58),
 * yawResponsePerNm(59),maxRpm(60),packetPeriodUs(61),packetDelayUs(62),
 * packetPhaseUs[3](63),outlierRpm(66),effectivenessExcludesReaction(must1)(67),
 * initialCounts(68),initialPWM(69),initialApproach(70),initialRpm[3](71).
 * Plant direction endpoints use controller's declared mathematical directions.
 */

enum simulationNumericColumn {
    SIM_REQUEST = 0, /* 4 */ SIM_ACTUAL_RESPONSE = 4, /* 4 */
    SIM_ACTUAL_ACTUATOR_RESPONSE = 8, /* 4 */
    SIM_TRUE_REACTION_NM = 12, SIM_TRUE_REACTION_RESPONSE = 13,
    SIM_TRUE_SERVO_COUNTS = 14, SIM_TRUE_DIRECTION = 15, SIM_TRUE_SERVO_DERIVATIVE = 16,
    SIM_TRUE_RPM = 17, /* 3 */ SIM_TRUE_ACCELERATION = 20, /* 3 */ SIM_TRUE_EFFORT = 23, /* 3 */
    SIM_COMMAND_RPM = 26, /* 3 */ SIM_COMMAND_EFFORT = 29, /* 3 */
    SIM_COMMAND_PWM = 32, SIM_TARGET_DIRECTION = 33, SIM_PREDICTED_DIRECTION = 34,
    SIM_PREDICTED_SERVO_COUNTS = 35, SIM_PREDICTED_SERVO_DERIVATIVE = 36,
    SIM_ESTIMATED_REACTION_NM = 37, SIM_APPLIED_REACTION_RESPONSE = 38,
    SIM_PREDICTED_ACTUATOR_RESPONSE = 39, /* 4 */ SIM_PREDICTED_TOTAL_RESPONSE = 43, /* 4 */
    SIM_COORDINATED_ACTUATOR_RESPONSE = 47, /* 4 */ SIM_COORDINATED_TOTAL_RESPONSE = 51, /* 4 */
    SIM_POST_COORDINATION_RESIDUAL = 55, /* 4 */ SIM_ALLOCATION_RESPONSE = 59, /* 4 */
    SIM_ALLOCATION_RESIDUAL = 63, /* 4 */ SIM_ALLOCATION_COST = 67, SIM_ALLOCATION_GRADIENT_NORM = 68,
    SIM_OBSERVER_RPM = 69, /* 3 */ SIM_OBSERVER_ACCELERATION = 72, /* 3 */
    SIM_PACKET_RPM = 75, /* 3 */ SIM_PREDICTED_CONTROLS = 78, /* 4 */
    SIM_CONTROLLER_PREDICTED_REACTION_RESPONSE = 82, SIM_TRUE_TAIL_VERTICAL = 83, SIM_TRUE_TAIL_RIGHTWARD = 84
};
enum simulationIntegerColumn {
    SIM_CONTROLLER_STATUS = 0, SIM_ALLOCATOR_STATUS = 1, SIM_ALLOCATOR_CONSTRAINTS = 2,
    SIM_ALLOCATOR_ITERATIONS = 3, SIM_TRUE_SERVO_STATUS = 4, SIM_REACTION_STATUS = 5,
    SIM_OBSERVER_STATUS = 6, /*3*/ SIM_OBSERVER_AGE = 9, /*3*/ SIM_OBSERVER_SEQUENCE = 12, /*3*/
    SIM_PACKET_OBSERVED_AT = 15, /*3 capture*/ SIM_PACKET_DELIVERED_AT = 18, /*3*/ SIM_PACKET_SEQUENCE = 21, /*3*/
    SIM_PACKET_QUEUE_DEPTH = 24 /*3*/
};
enum simulationFlagColumn {
    SIM_CONTROLLER_VALID = 0, SIM_ALLOCATOR_VALID = 1, SIM_ALLOCATOR_CONVERGED = 2, SIM_ALLOCATOR_EXACT = 3,
    SIM_SERVO_PREDICTION_AVAILABLE = 4, SIM_REACTION_AVAILABLE = 5,
    SIM_COORDINATED_ACTUATOR_AVAILABLE = 6, SIM_COORDINATED_TOTAL_AVAILABLE = 7,
    SIM_PREDICTED_ACTUATOR_AVAILABLE = 8, SIM_PREDICTED_TOTAL_AVAILABLE = 9,
    SIM_ACCELERATION_VALID = 10, /*3*/ SIM_RPM_SEEN = 13, /*3*/ SIM_RPM_ACCEPTED = 16, /*3*/
    SIM_RPM_FRESH = 19, /*3*/ SIM_RPM_VALID = 22, /*3*/ SIM_PACKET_SEEN = 25, /*3*/
    SIM_PACKET_GENERATED = 28, /*3*/ SIM_PACKET_DELIVERED = 31, /*3*/ SIM_PACKET_DROPPED = 34, /*3*/
    SIM_PACKET_OUTLIER = 37, /*3*/ SIM_TRUE_SERVO_VALID = 40, SIM_CALIBRATED = 41,
    SIM_FOR_FLIGHT = 42, SIM_TRUE_REACTION_VALID = 43, SIM_PLANT_RPM_CLIPPED = 44 /*3*/
};

/* Returns1 on success;0 malformed transport/config/time or queue overflow;
 * -1 if a supposedly synthetic model reports calibration/flight permission.
 * Float outputs never use NaN to represent unavailable estimates: consult flags.
 */
int tricopterSimulationReplay(const double *controller, const double *plant, size_t count,
    const uint64_t *timeUs, const float *requestedResponse, const uint8_t *modes,
    const uint8_t *dropGeneration, const uint8_t *outlierGeneration,
    double *numeric, uint32_t *integer, uint8_t *flags);
