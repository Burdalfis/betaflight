/* MATLAB transport to the same offline C simulation used by Python. */
#include "mex.h"
#include "simulation_bridge.h"

#include <float.h>
#include <math.h>
#include <stdint.h>

static const double *doubles(const mxArray *value, size_t count, const char *name)
{
    if (!mxIsDouble(value) || mxIsComplex(value) || mxIsSparse(value)
        || mxGetNumberOfElements(value) != count) {
        mexErrMsgIdAndTxt("tricopter:transport", "%s must be a full real double array with the declared size", name);
    }
    return mxGetPr(value);
}

static void shape(const mxArray *value, mwSize rows, mwSize columns, const char *name)
{
    if (mxGetNumberOfDimensions(value) != 2 || mxGetM(value) != rows || mxGetN(value) != columns) {
        mexErrMsgIdAndTxt("tricopter:shape", "%s has incorrect matrix dimensions", name);
    }
}

static void vector(const mxArray *value, const char *name)
{
    if (mxGetNumberOfDimensions(value) != 2 || (mxGetM(value) != 1 && mxGetN(value) != 1)) {
        mexErrMsgIdAndTxt("tricopter:shape", "%s must be a row or column vector", name);
    }
}

static void booleanRows(const mxArray *array, uint8_t *result, mwSize rows, mwSize columns, unsigned maximum)
{
    const double *source = doubles(array, rows * columns, "mode/fault array");
    shape(array, rows, columns, "mode/fault array");
    for (mwSize row = 0; row < rows; ++row) {
        for (mwSize column = 0; column < columns; ++column) {
            const double value = source[row + rows * column];
            if (!isfinite(value) || value < 0 || value > maximum || value != floor(value)) {
                mexErrMsgIdAndTxt("tricopter:transport", "Mode/fault metadata exceeds exact integer bounds");
            }
            result[row * columns + column] = (uint8_t)value;
        }
    }
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs != 7 || nlhs != 3) {
        mexErrMsgIdAndTxt("tricopter:usage", "[numeric,integer,flags] = tricopter_simulation_mex(controller,plant,timeUs,requests,modes,drop,outlier)");
    }
    const double *controller = doubles(prhs[0], SIMULATION_CONTROLLER_PARAMETERS, "controller parameters");
    const double *plant = doubles(prhs[1], SIMULATION_PLANT_PARAMETERS, "plant parameters");
    vector(prhs[0], "controller parameters"); vector(prhs[1], "plant parameters");
    vector(prhs[2], "timeUs");
    const mwSize rows = mxGetNumberOfElements(prhs[2]);
    if (!rows || rows > 1000000 || rows > SIZE_MAX / SIMULATION_NUMERIC_COLUMNS) {
        mexErrMsgIdAndTxt("tricopter:transport", "Expected 1..1000000 simulation ticks");
    }
    const double *time = doubles(prhs[2], rows, "timeUs");
    const double *request = doubles(prhs[3], rows * 4, "requests");
    shape(prhs[3], rows, 4, "requests");
    uint64_t *timeUs = mxCalloc(rows, sizeof(*timeUs));
    float *requests = mxCalloc(rows * 4, sizeof(*requests));
    uint8_t *modes = mxCalloc(rows, sizeof(*modes));
    uint8_t *drop = mxCalloc(rows * 3, sizeof(*drop));
    uint8_t *outlier = mxCalloc(rows * 3, sizeof(*outlier));
    for (mwSize row = 0; row < rows; ++row) {
        if (!isfinite(time[row]) || time[row] < 0 || time[row] > 9007199254740991.0 || time[row] != floor(time[row])) {
            mexErrMsgIdAndTxt("tricopter:transport", "timeUs must contain exact nonnegative integers below 2^53");
        }
        timeUs[row] = (uint64_t)time[row];
        for (mwSize column = 0; column < 4; ++column) {
            const double value = request[row + rows * column];
            if (!isfinite(value) || fabs(value) > FLT_MAX) {
                mexErrMsgIdAndTxt("tricopter:transport", "Requests must be finite float32 values");
            }
            requests[row * 4 + column] = (float)value;
        }
    }
    booleanRows(prhs[4], modes, rows, 1, 3);
    booleanRows(prhs[5], drop, rows, 3, 1);
    booleanRows(prhs[6], outlier, rows, 3, 1);
    double *numeric = mxCalloc(rows * SIMULATION_NUMERIC_COLUMNS, sizeof(*numeric));
    uint32_t *integer = mxCalloc(rows * SIMULATION_INTEGER_COLUMNS, sizeof(*integer));
    uint8_t *flags = mxCalloc(rows * SIMULATION_FLAG_COLUMNS, sizeof(*flags));
    const int result = tricopterSimulationReplay(controller, plant, rows, timeUs, requests, modes, drop, outlier, numeric, integer, flags);
    if (result != 1) {
        mexErrMsgIdAndTxt("tricopter:native", "Native simulation rejected transport/configuration/history (code %d)", result);
    }
    plhs[0] = mxCreateDoubleMatrix(rows, SIMULATION_NUMERIC_COLUMNS, mxREAL);
    plhs[1] = mxCreateNumericMatrix(rows, SIMULATION_INTEGER_COLUMNS, mxUINT32_CLASS, mxREAL);
    plhs[2] = mxCreateNumericMatrix(rows, SIMULATION_FLAG_COLUMNS, mxUINT8_CLASS, mxREAL);
    double *numericOut = mxGetPr(plhs[0]);
    uint32_t *integerOut = mxGetData(plhs[1]);
    uint8_t *flagsOut = mxGetData(plhs[2]);
    for (mwSize row = 0; row < rows; ++row) {
        for (mwSize column = 0; column < SIMULATION_NUMERIC_COLUMNS; ++column) {
            const double value = numeric[row * SIMULATION_NUMERIC_COLUMNS + column];
            if (!isfinite(value)) mexErrMsgIdAndTxt("tricopter:numeric", "Nonfinite native output");
            numericOut[row + rows * column] = value;
        }
        for (mwSize column = 0; column < SIMULATION_INTEGER_COLUMNS; ++column) {
            integerOut[row + rows * column] = integer[row * SIMULATION_INTEGER_COLUMNS + column];
        }
        for (mwSize column = 0; column < SIMULATION_FLAG_COLUMNS; ++column) {
            flagsOut[row + rows * column] = flags[row * SIMULATION_FLAG_COLUMNS + column];
        }
    }
    mxFree(timeUs); mxFree(requests); mxFree(modes); mxFree(drop); mxFree(outlier);
    mxFree(numeric); mxFree(integer); mxFree(flags);
}
