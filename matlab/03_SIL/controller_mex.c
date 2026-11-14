/* controller_mex.c
 * Wrapper MEX supaya controller_step() bisa dipanggil langsung dari
 * MATLAB, tanpa perlu compiler command-line terpisah -- pakai 'mex'
 * yang sudah terkonfigurasi ke Visual C++ 2022.
 *
 * Compile di MATLAB Command Window (folder yang sama dgn file2 ini):
 *   mex controller_mex.c controller_step.c
 *
 * Pakai:
 *   y_c = controller_mex(e);   % e = vektor kolom/baris double
 */
#include "mex.h"
#include "controller_step.h"

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    double *e, *y;
    mwSize n, i;

    if (nrhs != 1) {
        mexErrMsgTxt("Pakai: y = controller_mex(e)");
    }
    e = mxGetPr(prhs[0]);
    n = mxGetNumberOfElements(prhs[0]);

    plhs[0] = mxCreateDoubleMatrix(n, 1, mxREAL);
    y = mxGetPr(plhs[0]);

    controller_init();
    for (i = 0; i < n; i++) {
        /* e_dalam = vref - vo_meas = e[i] - 0 = e[i] */
        y[i] = (double) controller_step((float) e[i], 0.0f);
    }
}
