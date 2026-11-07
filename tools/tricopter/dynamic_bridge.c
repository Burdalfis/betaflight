/* Opt-in coordinator policy sharing the unchanged synthetic plant engine. */
#include "dynamic_bridge.h"
#include <float.h>
#include <math.h>
#include <string.h>

typedef struct dynamicContext_s {
    tricopterSimulationState_t state;
    tricopterDynamicAllocatorParams_t params;
    double *numeric,*candidates;
    uint32_t *integer,*candidateInteger;
} dynamicContext_t;

static bool readDynamic(const double *v,tricopterDynamicAllocatorParams_t *p)
{
    for(unsigned i=0;i<DYNAMIC_PARAMETERS;++i) if(!isfinite(v[i])||fabs(v[i])>FLT_MAX) return false;
    if(v[0]<0||v[0]>UINT32_MAX||v[0]!=floor(v[0])||v[1]<0||v[1]>UINT32_MAX||v[1]!=floor(v[1])
        ||v[31]<0||v[31]>17||v[31]!=floor(v[31])) return false;
    memset(p,0,sizeof(*p));p->horizonUs=(uint32_t)v[0];p->maxRpmAgeUs=(uint32_t)v[1];
    for(unsigned i=0;i<3;++i){p->motorTauIncreasingS[i]=(float)v[2+i];p->motorTauDecreasingS[i]=(float)v[5+i];
        p->motorSlewIncreasingRpmS[i]=(float)v[8+i];p->motorSlewDecreasingRpmS[i]=(float)v[11+i];}
    for(unsigned i=0;i<17;++i)p->pwmGridUs[i]=(float)v[14+i];
    p->gridCount=(uint8_t)v[31];p->maxHighMatrixCondition=(float)v[32];p->highCostTolerance=(float)v[33];
    p->yawCostTolerance=(float)v[34];p->movementCostTolerance=(float)v[35];return true;
}
static bool resetPolicy(void *context,const tricopterSimulationParams_t *base,const tricopterSimulationInitialState_t *initial)
{
    dynamicContext_t *c=context;tricopterSimulationParams_t params=*base;
    params.dynamicConfigured=true;params.dynamic=c->params;
    return tricopterSimulationReset(&c->state,&params,initial);
}
static tricopterSimulationOutput_t tickPolicy(void *context,uint64_t now,const tricopterSimulationInput_t *input)
{
    dynamicContext_t *c=context;return tricopterSimulationTick(&c->state,now,input);
}
static void candidateValues(double *f,uint32_t *u,const tricopterDynamicAllocatorCandidate_t *v)
{
    f[DYNC_PWM]=v->pwmUs;f[DYNC_COUNTS_H]=v->horizonCorrectedCounts;f[DYNC_DIRECTION_H]=v->horizonDirectionRad;
    for(unsigned i=0;i<3;++i){f[DYNC_CONTINUOUS_EFFORT+i]=v->continuousOptimalEffort[i];f[DYNC_COMMAND_RPM+i]=v->commandMechanicalRpm[i];
        f[DYNC_RPM_H+i]=v->predictedMechanicalRpm[i];f[DYNC_EFFORT_H+i]=v->predictedEffort[i];}
    for(unsigned i=0;i<4;++i){f[DYNC_CONTROLS_H+i]=v->predictedControls[i];f[DYNC_RESPONSE_H+i]=v->predictedResponse[i];f[DYNC_RESIDUAL_H+i]=v->residualResponse[i];}
    f[DYNC_HIGH]=v->highCost;f[DYNC_YAW]=v->yawCost;f[DYNC_MOVE]=v->movementCost;f[DYNC_CONDITION]=v->highMatrixCondition;
    u[0]=v->status;u[1]=v->evaluatedFaces;u[2]=v->valid;
}
static void recordPolicy(void *context,size_t row,const tricopterSimulationOutput_t *output)
{
    dynamicContext_t *c=context;const tricopterDynamicAllocatorOutput_t *v=&output->dynamicAllocation;
    double *f=c->numeric+row*DYNAMIC_NUMERIC_COLUMNS;uint32_t *u=c->integer+row*DYNAMIC_INTEGER_COLUMNS;
    memset(f,0,DYNAMIC_NUMERIC_COLUMNS*sizeof(*f));memset(u,0,DYNAMIC_INTEGER_COLUMNS*sizeof(*u));
    f[DYN_PWM]=v->selected.pwmUs;f[DYN_DIRECTION_H]=v->selected.horizonDirectionRad;
    f[DYN_MIN_HIGH]=v->minimumHighCost;f[DYN_MIN_YAW]=v->minimumYawCostWithinHighTolerance;
    f[DYN_SELECTED_HIGH]=v->selected.highCost;f[DYN_SELECTED_YAW]=v->selected.yawCost;f[DYN_SELECTED_MOVE]=v->selected.movementCost;
    f[DYN_COUNTS_H]=v->selected.horizonCorrectedCounts;f[DYN_MIN_PWM]=v->minimumPwmUs;f[DYN_MAX_PWM]=v->maximumPwmUs;
    for(unsigned i=0;i<3;++i){f[DYN_REACHABLE_MIN+i]=v->reachableMinEffort[i];f[DYN_REACHABLE_MAX+i]=v->reachableMaxEffort[i];
        f[DYN_COMMAND_MIN+i]=v->minimumCommandRpm[i];f[DYN_COMMAND_MAX+i]=v->maximumCommandRpm[i];
        f[DYN_EFFORT_H+i]=v->selected.predictedEffort[i];f[DYN_RPM_H+i]=v->selected.predictedMechanicalRpm[i];f[DYN_COMMAND_RPM+i]=v->selected.commandMechanicalRpm[i];}
    for(unsigned i=0;i<4;++i){f[DYN_RESPONSE_H+i]=v->selected.predictedResponse[i];f[DYN_RESIDUAL_H+i]=v->selected.residualResponse[i];f[DYN_NET_REQUEST+i]=output->netRequestedResponse[i];}
    u[DYN_STATUS]=v->status;u[DYN_CANDIDATE_COUNT]=v->candidateCount;u[DYN_VALID_CANDIDATE_COUNT]=v->validCandidateCount;
    u[DYN_SELECTED_INDEX]=v->selectedCandidate;u[DYN_VALID]=v->valid;u[DYN_PRIORITY_AVAILABLE]=v->priorityAvailable;
    u[DYN_CALIBRATED]=v->calibrated;u[DYN_FOR_FLIGHT]=v->forFlight;u[DYN_USED]=output->dynamicAllocationUsed;
    for(unsigned candidate=0;candidate<DYNAMIC_MAX_CANDIDATES;++candidate){
        double *cf=c->candidates+(row*DYNAMIC_MAX_CANDIDATES+candidate)*DYNAMIC_CANDIDATE_COLUMNS;
        uint32_t *cu=c->candidateInteger+(row*DYNAMIC_MAX_CANDIDATES+candidate)*DYNAMIC_CANDIDATE_INTEGER_COLUMNS;
        memset(cf,0,DYNAMIC_CANDIDATE_COLUMNS*sizeof(*cf));memset(cu,0,DYNAMIC_CANDIDATE_INTEGER_COLUMNS*sizeof(*cu));
        if(candidate<v->candidateCount)candidateValues(cf,cu,&v->candidates[candidate]);
    }
}
int tricopterDynamicSimulationReplay(const double *controller,const double *plant,const double *dynamic,size_t count,
    const uint64_t *timeUs,const float *request,const uint8_t *modes,const uint8_t *drop,const uint8_t *outlier,
    double *numeric,uint32_t *integer,uint8_t *flags,double *dynamicNumeric,uint32_t *dynamicInteger,
    double *candidateNumeric,uint32_t *candidateInteger)
{
    if(!dynamic||!dynamicNumeric||!dynamicInteger||!candidateNumeric||!candidateInteger||count>SIZE_MAX/(DYNAMIC_MAX_CANDIDATES*DYNAMIC_CANDIDATE_COLUMNS))return 0;
    dynamicContext_t context={.numeric=dynamicNumeric,.integer=dynamicInteger,.candidates=candidateNumeric,.candidateInteger=candidateInteger};
    if(!readDynamic(dynamic,&context.params))return 0;
    const simulationReplayPolicy_t policy={.context=&context,.reset=resetPolicy,.tick=tickPolicy,.record=recordPolicy};
    return tricopterSimulationReplayWithPolicy(controller,plant,count,timeUs,request,modes,drop,outlier,numeric,integer,flags,&policy);
}
