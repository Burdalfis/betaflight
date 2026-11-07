/* Opt-in dynamic policy transport. The original simulation ABI is unchanged. */
#pragma once
#include "simulation_bridge.h"

#define DYNAMIC_PARAMETERS 36U
#define DYNAMIC_NUMERIC_COLUMNS 43U
#define DYNAMIC_INTEGER_COLUMNS 9U
#define DYNAMIC_CANDIDATE_COLUMNS 31U
#define DYNAMIC_CANDIDATE_INTEGER_COLUMNS 3U
#define DYNAMIC_MAX_CANDIDATES 19U

/* dynamic double[36]: H_us(0),maxRpmAge_us(1),tauInc[3](2),tauDec[3](5),
 * slewInc[3](8),slewDec[3](11),PWMgrid[17](14),gridCount(31),
 * highMatrixConditionGuard(32),highTolerance(33),yawTolerance(34),moveTolerance(35).
 * Source/config provenance remains synthetic-only through controller/plant.
 * Extra arrays are row major. Candidate arrays are [rows,19,31] and [rows,19,3].
 */
enum dynamicNumericColumn {
    DYN_PWM=0,DYN_DIRECTION_H=1,DYN_MIN_HIGH=2,DYN_MIN_YAW=3,
    DYN_SELECTED_HIGH=4,DYN_SELECTED_YAW=5,DYN_SELECTED_MOVE=6,
    DYN_REACHABLE_MIN=7, /*3*/ DYN_REACHABLE_MAX=10, /*3*/
    DYN_COMMAND_MIN=13, /*3*/ DYN_COMMAND_MAX=16, /*3*/
    DYN_EFFORT_H=19, /*3*/ DYN_RPM_H=22, /*3*/ DYN_RESPONSE_H=25, /*4*/
    DYN_RESIDUAL_H=29, /*4*/ DYN_COMMAND_RPM=33, /*3*/ DYN_COUNTS_H=36,
    DYN_NET_REQUEST=37, /*4; authoritative published coordinator net response*/
    DYN_MIN_PWM=41,DYN_MAX_PWM=42
};
enum dynamicIntegerColumn {
    DYN_STATUS=0,DYN_CANDIDATE_COUNT=1,DYN_VALID_CANDIDATE_COUNT=2,DYN_SELECTED_INDEX=3,
    DYN_VALID=4,DYN_PRIORITY_AVAILABLE=5,DYN_CALIBRATED=6,DYN_FOR_FLIGHT=7,DYN_USED=8
};
enum dynamicCandidateColumn {
    DYNC_PWM=0,DYNC_COUNTS_H=1,DYNC_DIRECTION_H=2,DYNC_CONTINUOUS_EFFORT=3, /*3*/
    DYNC_COMMAND_RPM=6, /*3*/ DYNC_RPM_H=9, /*3*/ DYNC_EFFORT_H=12, /*3*/
    DYNC_CONTROLS_H=15, /*4*/ DYNC_RESPONSE_H=19, /*4*/ DYNC_RESIDUAL_H=23, /*4*/
    DYNC_HIGH=27,DYNC_YAW=28,DYNC_MOVE=29,DYNC_CONDITION=30
};
/* Candidate integer columns: status, evaluatedFaces, valid. */
int tricopterDynamicSimulationReplay(const double *controller,const double *plant,const double *dynamic,size_t count,
    const uint64_t *timeUs,const float *requestedResponse,const uint8_t *modes,const uint8_t *dropGeneration,const uint8_t *outlierGeneration,
    double *numeric,uint32_t *integer,uint8_t *flags,double *dynamicNumeric,uint32_t *dynamicInteger,
    double *candidateNumeric,uint32_t *candidateInteger);
