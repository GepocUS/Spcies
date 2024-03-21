#include "mex.h"
#include "$C_CODE_NAME$.h"
#include <math.h>
#include <string.h>

// Matlab MEX function

void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]){

    // Variable declaration
    double *x0; // Local x0
    double *xre; // Local xre
    double *xrs; // Local xrs
    double *xrc; // Local xrc
    double *ure; // Local ure
    double *urs; // Local urs
    double *urc; // Local urc
    double *u_opt; // Local u_opt
    int k; // Local k
    int e_flag; // Local e_flag
    sol_$C_CODE_NAME$ sol = {.update_time = 0.0, .solve_time = 0.0, .polish_time = 0.0, .run_time = 0.0};

    // Check inputs and outputs

    // Check number of inputs
    if(nrhs != 7){
        mexErrMsgIdAndTxt("Spcies:HMPC:nrhs:number",
                          "Not enough inputs");
    }

    // Check number of outputs
    if(nlhs == 0){
        mexErrMsgIdAndTxt("Spcies:HMPC:nlhs:number",
                          "At least one output required");
    }

    // Check that x0 is of the correct dimension
    if( !mxIsDouble(prhs[0]) || mxGetNumberOfElements(prhs[0]) != nn_ ){
        mexErrMsgIdAndTxt("Spcies:HMPC:nrhs:x0",
                          "x0 must be of dimension nn");
    }
    // Check that xre is of the correct dimension
    if( !mxIsDouble(prhs[1]) || mxGetNumberOfElements(prhs[1]) != nn_ ){
        mexErrMsgIdAndTxt("Spcies:HMPC:nrhs:xre",
                          "xre must be of dimension nn");
    }
    // Check that xrs is of the correct dimension
    if( !mxIsDouble(prhs[2]) || mxGetNumberOfElements(prhs[2]) != nn_ ){
        mexErrMsgIdAndTxt("Spcies:HMPC:nrhs:xrs",
                          "xrs must be of dimension nn");
    }
    // Check that xrc is of the correct dimension
    if( !mxIsDouble(prhs[3]) || mxGetNumberOfElements(prhs[3]) != nn_ ){
        mexErrMsgIdAndTxt("Spcies:HMPC:nrhs:xrc",
                          "xrc must be of dimension nn");
    }
    // Check that ure is of the correct dimension
    if( !mxIsDouble(prhs[4]) || mxGetNumberOfElements(prhs[4]) != mm_ ){
        mexErrMsgIdAndTxt("Spcies:HMPC:nrhs:ure",
                          "ure must be of dimension mm");
    }
    // Check that urs is of the correct dimension
    if( !mxIsDouble(prhs[5]) || mxGetNumberOfElements(prhs[5]) != mm_ ){
        mexErrMsgIdAndTxt("Spcies:HMPC:nrhs:urs",
                          "urs must be of dimension mm");
    }
    // Check that urc is of the correct dimension
    if( !mxIsDouble(prhs[6]) || mxGetNumberOfElements(prhs[6]) != mm_ ){
        mexErrMsgIdAndTxt("Spcies:HMPC:nrhs:urc",
                          "urc must be of dimension mm");
    }

    // Read input data
    x0 = (double*) mxGetData(prhs[0]);
    xre = (double*) mxGetData(prhs[1]);
    xrs = (double*) mxGetData(prhs[2]);
    xrc = (double*) mxGetData(prhs[3]);
    ure = (double*) mxGetData(prhs[4]);
    urs = (double*) mxGetData(prhs[5]);
    urc = (double*) mxGetData(prhs[6]);

    // Prepare output data
    mxArray *z_pt, *s_pt, *lambda_pt, *update_time_pt, *solve_time_pt, *polish_time_pt, *run_time_pt;
    double *z_out, *s_out, *lambda_out, *update_time_out, *solve_time_out, *polish_time_out, *run_time_out, *k_out, *e_flag_out;

    plhs[0] = mxCreateDoubleMatrix(mm_, 1, mxREAL); // u_opt
    plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL); // k
    plhs[2] = mxCreateDoubleMatrix(1, 1, mxREAL); // e_flag

    u_opt = (double*) mxGetData(plhs[0]);
    k_out = (double*) mxGetData(plhs[1]);
    e_flag_out = (double*) mxGetData(plhs[2]);

    // Solution structure
    const char *field_names[] = {"z", "s", "lambda", "update_time", "solve_time", "polish_time", "run_time"};
    plhs[3] = mxCreateStructMatrix(1, 1, 7, field_names);

    z_pt = mxCreateDoubleMatrix(dim, 1, mxREAL);
    s_pt = mxCreateDoubleMatrix(n_s, 1, mxREAL);
    lambda_pt = mxCreateDoubleMatrix(n_s, 1, mxREAL);
    update_time_pt = mxCreateDoubleMatrix(1, 1, mxREAL);
    solve_time_pt = mxCreateDoubleMatrix(1, 1, mxREAL);
    polish_time_pt = mxCreateDoubleMatrix(1, 1, mxREAL);
    run_time_pt = mxCreateDoubleMatrix(1, 1, mxREAL);

    z_out = (double*) mxGetData(z_pt);
    s_out = (double*) mxGetData(s_pt);
    lambda_out = (double*) mxGetData(lambda_pt);
    update_time_out = (double*) mxGetData(update_time_pt);
    solve_time_out = (double*) mxGetData(solve_time_pt);
    polish_time_out = (double*) mxGetData(polish_time_pt);
    run_time_out = (double*) mxGetData(run_time_pt);

    mxSetField(plhs[3], 0, "z", z_pt);
    mxSetField(plhs[3], 0, "s", s_pt);
    mxSetField(plhs[3], 0, "lambda", lambda_pt);
    mxSetField(plhs[3], 0, "update_time", update_time_pt);
    mxSetField(plhs[3], 0, "solve_time", solve_time_pt);
    mxSetField(plhs[3], 0, "polish_time", polish_time_pt);
    mxSetField(plhs[3], 0, "run_time", run_time_pt);

    // Call solver
    HMPC_ADMM(x0, xre, xrs, xrc, ure, urs, urc, u_opt, &k, &e_flag, &sol);

    // Set output values
    *k_out = (double) k;
    *e_flag_out = (double) e_flag;
    *update_time_out = sol.update_time;
    *solve_time_out = sol.solve_time;
    *polish_time_out = sol.polish_time;
    *run_time_out = sol.run_time;

    #ifdef DEBUG

    for (unsigned int i = 0; i < dim; i++) {
        z_out[i] = sol.z[i];
    }
    for (unsigned int i = 0; i < n_s; i++) {
        s_out[i] = sol.s[i];
        lambda_out[i] = sol.lambda[i];
    }

    #endif

}

