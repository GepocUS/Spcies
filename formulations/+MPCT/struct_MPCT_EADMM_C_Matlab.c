#include "mex.h"
#include "$C_CODE_NAME$.h"
#include <math.h>
#include <string.h>

// Matlab MEX function

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]){

    // Variable declaration
    double *x0; // Local x0
    double *xr; // Local xr
    double *ur; // Local ur
    double *u_opt; // Local u_opt
    int k; // Local k
    int e_flag; // Local e_flag
    sol_$C_CODE_NAME$ sol = {.update_time = 0.0, .solve_time = 0.0, .polish_time = 0.0, .run_time = 0.0};

    // Check inputs and outputs

    // Check number of inputs
    if(nrhs != 3){
        mexErrMsgIdAndTxt("Spcies:MPCT_EADMM:nrhs:number",
                          "Not enough inputs");
    }

    // Check number of outputs
    if(nlhs == 0){
        mexErrMsgIdAndTxt("Spcies:MPCT_EADMM:nlhs:number",
                          "At least one output required");
    }

    // Check that x0 is of the correct dimension
    if( !mxIsDouble(prhs[0]) || mxGetNumberOfElements(prhs[0]) != nn_ ){
        mexErrMsgIdAndTxt("Spcies:MPCT_EADMM:nrhs:x0",
                          "x0 must be of dimension nn");
    }
    // Check that xr is of the correct dimension
    if( !mxIsDouble(prhs[1]) || mxGetNumberOfElements(prhs[1]) != nn_ ){
        mexErrMsgIdAndTxt("Spcies:MPCT_EADMM:nrhs:xr",
                          "xr must be of dimension nn");
    }
    // Check that ur is of the correct dimension
    if( !mxIsDouble(prhs[2]) || mxGetNumberOfElements(prhs[2]) != mm_ ){
        mexErrMsgIdAndTxt("Spcies:MPCT_EADMM:nrhs:ur",
                          "ur must be of dimension mm");
    }

    // Read input data
    x0 = (double*) mxGetData(prhs[0]);
    xr = (double*) mxGetData(prhs[1]);
    ur = (double*) mxGetData(prhs[2]);

    // Prepare output data
    mxArray *z1_pt, *z2_pt, *z3_pt, *lambda_pt, *update_time_pt, *solve_time_pt, *polish_time_pt, *run_time_pt;
    double *z1_out, *z2_out, *z3_out, *lambda_out, *update_time_out, *solve_time_out, *polish_time_out, *run_time_out, *k_out, *e_flag_out;

    plhs[0] = mxCreateDoubleMatrix(mm_, 1, mxREAL); // u_opt
    plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL); // k
    plhs[2] = mxCreateDoubleMatrix(1, 1, mxREAL); // e_flag

    u_opt = (double*) mxGetData(plhs[0]);
    k_out = (double*) mxGetData(plhs[1]);
    e_flag_out = (double*) mxGetData(plhs[2]);

    // Solution structure
    const char *field_names[] = {"z1", "z2", "z3", "lambda", "update_time", "solve_time", "polish_time", "run_time"};
    plhs[3] = mxCreateStructMatrix(1, 1, 8, field_names);

    z1_pt = mxCreateDoubleMatrix((NN_+1)*nm_, 1, mxREAL);
    z2_pt = mxCreateDoubleMatrix(nm_, 1, mxREAL);
    z3_pt = mxCreateDoubleMatrix((NN_+1)*nm_, 1, mxREAL);
    lambda_pt = mxCreateDoubleMatrix((NN_+3)*nm_, 1, mxREAL);
    update_time_pt = mxCreateDoubleMatrix(1, 1, mxREAL);
    solve_time_pt = mxCreateDoubleMatrix(1, 1, mxREAL);
    polish_time_pt = mxCreateDoubleMatrix(1, 1, mxREAL);
    run_time_pt = mxCreateDoubleMatrix(1, 1, mxREAL);

    z1_out = (double*) mxGetData(z1_pt);
    z2_out = (double*) mxGetData(z2_pt);
    z3_out = (double*) mxGetData(z3_pt);
    lambda_out = (double*) mxGetData(lambda_pt);
    update_time_out = (double*) mxGetData(update_time_pt);
    solve_time_out = (double*) mxGetData(solve_time_pt);
    polish_time_out = (double*) mxGetData(polish_time_pt);
    run_time_out = (double*) mxGetData(run_time_pt);

    mxSetField(plhs[3], 0, "z1", z1_pt);
    mxSetField(plhs[3], 0, "z2", z2_pt);
    mxSetField(plhs[3], 0, "z3", z3_pt);
    mxSetField(plhs[3], 0, "lambda", lambda_pt);
    mxSetField(plhs[3], 0, "update_time", update_time_pt);
    mxSetField(plhs[3], 0, "solve_time", solve_time_pt);
    mxSetField(plhs[3], 0, "polish_time", polish_time_pt);
    mxSetField(plhs[3], 0, "run_time", run_time_pt);

    // Call solver
    MPCT_EADMM(x0, xr, ur, u_opt, &k, &e_flag, &sol);

    // Set output values
    *k_out = (double) k;
    *e_flag_out = (double) e_flag;
    *update_time_out = sol.update_time;
    *solve_time_out = sol.solve_time;
    *polish_time_out = sol.polish_time;
    *run_time_out = sol.run_time;

    #ifdef DEBUG

    for (unsigned int i = 0; i < (NN_+1)*nm_; i++) {
        z1_out[i] = sol.z1[i];
        z3_out[i] = sol.z3[i];
    }

    for (unsigned int i = 0; i < nm_; i++) {
        z2_out[i] = sol.z2[i];
    }

    for (unsigned int i = 0; i < (NN_+3)*nm_; i++) {
        lambda_out[i] = sol.lambda[i];
    }

    #endif

}

