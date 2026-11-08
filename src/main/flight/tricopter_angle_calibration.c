#include "tricopter_angle_calibration.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static bool finiteFloat(float value)
{
    uint32_t bits; memcpy(&bits, &value, sizeof(bits));
    return (bits & UINT32_C(0x7f800000)) != UINT32_C(0x7f800000);
}

bool tricopterAngleCalibrationValidate(const tricopterAngleCalibration_t *m)
{
    if (!m || m->knotCount < 2 || m->knotCount > TRICOPTER_ANGLE_CALIBRATION_MAX_KNOTS
        || !m->operatorDefinedReference) return false;
    const bool increasing = m->angleRad[1] > m->angleRad[0];
    for (unsigned i = 0; i < m->knotCount; ++i) {
        if (!finiteFloat(m->correctedCounts[i]) || fabsf(m->correctedCounts[i]) > 4095
            || !finiteFloat(m->angleRad[i]) || fabsf(m->angleRad[i]) > 3.141593f
            || !finiteFloat(m->halfBranchSeparationRad[i]) || m->halfBranchSeparationRad[i] < 0
            || m->halfBranchSeparationRad[i] > 3.141593f) return false;
        if (i && (m->correctedCounts[i] <= m->correctedCounts[i-1]
            || (increasing ? m->angleRad[i] <= m->angleRad[i-1] : m->angleRad[i] >= m->angleRad[i-1]))) return false;
    }
    return true;
}

tricopterAngleCalibrationOutput_t tricopterAngleCalibrationEvaluate(const tricopterAngleCalibration_t *m, float counts)
{
    tricopterAngleCalibrationOutput_t out = {0};
    if (!tricopterAngleCalibrationValidate(m) || !finiteFloat(counts)
        || counts < m->correctedCounts[0] || counts > m->correctedCounts[m->knotCount-1]) return out;
    for (unsigned i = 1; i < m->knotCount; ++i) {
        if (counts > m->correctedCounts[i]) continue;
        const double t = ((double)counts-m->correctedCounts[i-1]) / ((double)m->correctedCounts[i]-m->correctedCounts[i-1]);
        out.angleRad = (float)(m->angleRad[i-1] + t*((double)m->angleRad[i]-m->angleRad[i-1]));
        out.halfBranchSeparationRad = (float)(m->halfBranchSeparationRad[i-1] + t*((double)m->halfBranchSeparationRad[i]-m->halfBranchSeparationRad[i-1]));
        out.valid = out.operatorDefinedReference = true;
        return out;
    }
    return out;
}

bool tricopterAngleCalibrationInverse(const tricopterAngleCalibration_t *m, float angle, float *counts)
{
    if (!counts || !tricopterAngleCalibrationValidate(m) || !finiteFloat(angle)) return false;
    for (unsigned i = 1; i < m->knotCount; ++i) {
        const double a=m->angleRad[i-1], b=m->angleRad[i];
        if (angle < fmin(a,b) || angle > fmax(a,b)) continue;
        const double t=((double)angle-a)/(b-a);
        *counts=(float)(m->correctedCounts[i-1]+t*((double)m->correctedCounts[i]-m->correctedCounts[i-1]));
        return true;
    }
    return false;
}
