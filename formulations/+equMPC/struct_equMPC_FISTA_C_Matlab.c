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
    #if TIME_VARYING == 1
    double *A;
    double *B;
    double *Q;
    double *R;
    double *LB;
    double *UB;
    #endif
    double *u_opt; // Local u_opt
    int k; // Local k
    int e_flag; // Local e_flag
    sol_$C_CODE_NAME$ sol = {.update_time = 0.0, .solve_time = 0.0, .polish_time = 0.0, .run_time = 0.0};

    // Check inputs and outputs

    #if TIME_VARYING == 1
    if(nrhs != 9){
        mexErrMsgIdAndTxt("Spcies:equMPC:nrhs:number", "Nine inputs are required");
    }
    #else
    if(nrhs != 3){
        mexErrMsgIdAndTxt("Spcies:equMPC:nrhs:number", "Three inputs are required");
    }
    #endif

    // Check number of outputs
    if(nlhs == 0){
        mexErrMsgIdAndTxt("Spcies:equMPC:nlhs:number", "At least one output is required");
    }

    // Check that x0 is of the correct dimension
    if( !mxIsDouble(prhs[0]) || mxGetNumberOfElements(prhs[0]) != nn_ ){
        mexErrMsgIdAndTxt("Spcies:equMPC:nrhs:x0", "x0 must be of dimension %%d", nn_);
    }
    // Check that xr is of the correct dimension
    if( !mxIsDouble(prhs[1]) || mxGetNumberOfElements(prhs[1]) != nn_ ){
        mexErrMsgIdAndTxt("Spcies:equMPC:nrhs:xr", "xr must be of dimension %%d", nn_);
    }
    // Check that ur is of the correct dimension
    if( !mxIsDouble(prhs[2]) || mxGetNumberOfElements(prhs[2]) != mm_ ){
        mexErrMsgIdAndTxt("Spcies:equMPC:nrhs:ur", "ur must be of dimension %%d", mm_);
    }

    #if TIME_VARYING == 1

    if( !mxIsDouble(prhs[3]) || mxGetNumberOfElements(prhs[3]) != nn_*nn_ ){
        mexErrMsgIdAndTxt("Spcies:equMPC:nrhs:A", "A must be of dimension %%d by %%d", nn_, nn_);
    }
    
    if( !mxIsDouble(prhs[4]) || mxGetNumberOfElements(prhs[4]) != nn_*mm_ ){
        mexErrMsgIdAndTxt("Spcies:equMPC:nrhs:B", "B must be of dimension %%d by %%d", nn_, mm_);
    }

    if( !mxIsDouble(prhs[5]) || mxGetNumberOfElements(prhs[5]) != nn_ ){
        mexErrMsgIdAndTxt("Spcies:equMPC:nrhs:Q", "Q must be a diagonal vector of %%d elements", nn_);
    }
    
    if( !mxIsDouble(prhs[6]) || mxGetNumberOfElements(prhs[6]) != mm_ ){
        mexErrMsgIdAndTxt("Spcies:equMPC:nrhs:R", "R must be a diagonal vector of %%d elements", mm_);
    }

    if( !mxIsDouble(prhs[7]) || mxGetNumberOfElements(prhs[7]) != nm_ ){
        mexErrMsgIdAndTxt("Spcies:equMPC:nrhs:LB", "LB must be of dimension %%d", nm_);
    }

    if( !mxIsDouble(prhs[8]) || mxGetNumberOfElements(prhs[8]) != nm_ ){
        mexErrMsgIdAndTxt("Spcies:equMPC:nrhs:UB", "UB must be of dimension %%d", nm_);
    }
    #endif

    // Read input data
    x0 = (double*) mxGetData(prhs[0]);
    xr = (double*) mxGetData(prhs[1]);
    ur = (double*) mxGetData(prhs[2]);

    #if TIME_VARYING == 1
    A = (double*) mxGetData(prhs[3]);
    B = (double*) mxGetData(prhs[4]);
    Q = (double*) mxGetData(prhs[5]);
    R = (double*) mxGetData(prhs[6]);
    LB = (double*) mxGetData(prhs[7]);
    UB = (double*) mxGetData(prhs[8]);
    #endif

    // Prepare output data
    mxArray *z_pt, *lambda_pt, *update_time_pt, *solve_time_pt, *polish_time_pt, *run_time_pt;
    double *z_out, *lambda_out, *update_time_out, *solve_time_out, *polish_time_out, *run_time_out, *k_out, *e_flag_out;

    plhs[0] = mxCreateDoubleMatrix(mm_, 1, mxREAL); // u_opt
    plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL); // k
    plhs[2] = mxCreateDoubleMatrix(1, 1, mxREAL); // e_flag

    u_opt = (double*) mxGetData(plhs[0]);
    k_out = (double*) mxGetData(plhs[1]);
    e_flag_out = (double*) mxGetData(plhs[2]);

    const char *field_names[] = {"z", "lambda", "update_time", "solve_time", "polish_time", "run_time"};
    plhs[3] = mxCreateStructMatrix(1, 1, 6, field_names);

    z_pt = mxCreateDoubleMatrix((NN_-1)*nm_+mm_, 1, mxREAL);
    lambda_pt = mxCreateDoubleMatrix(NN_*nn_, 1, mxREAL);
    update_time_pt = mxCreateDoubleMatrix(1, 1, mxREAL);
    solve_time_pt = mxCreateDoubleMatrix(1, 1, mxREAL);
    polish_time_pt = mxCreateDoubleMatrix(1, 1, mxREAL);
    run_time_pt = mxCreateDoubleMatrix(1, 1, mxREAL);

    z_out = (double*) mxGetData(z_pt);
    lambda_out = (double*) mxGetData(lambda_pt);
    update_time_out = (double*) mxGetData(update_time_pt);
    solve_time_out = (double*) mxGetData(solve_time_pt);
    polish_time_out = (double*) mxGetData(polish_time_pt);
    run_time_out = (double*) mxGetData(run_time_pt);

    mxSetField(plhs[3], 0, "z", z_pt);
    mxSetField(plhs[3], 0, "lambda", lambda_pt);
    mxSetField(plhs[3], 0, "update_time", update_time_pt);
    mxSetField(plhs[3], 0, "solve_time", solve_time_pt);
    mxSetField(plhs[3], 0, "polish_time", polish_time_pt);
    mxSetField(plhs[3], 0, "run_time", run_time_pt);

    // Call solver
    #if TIME_VARYING == 1
    equMPC_FISTA(x0, xr, ur, A, B, Q, R, LB, UB, u_opt, &k, &e_flag, &sol);
    #else
    equMPC_FISTA(x0, xr, ur, u_opt, &k, &e_flag, &sol);
    #endif

    // Set output values
    *k_out = (double) k;
    *e_flag_out = (double) e_flag;
    *update_time_out = sol.update_time;
    *solve_time_out = sol.solve_time;
    *polish_time_out = sol.polish_time;
    *run_time_out = sol.run_time;

    #ifdef DEBUG

    for (unsigned int i = 0; i < (NN_-1)*nm_+mm_; i++) {
        z_out[i] = sol.z[i];
    }
    for (unsigned int i = 0; i < NN_*nn_; i++) {
        lambda_out[i] = sol.lambda[i];
    }

    #endif
}

