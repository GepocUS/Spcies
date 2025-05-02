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
    #if SOFT_CONSTRAINTS == 1 && ADAPTIVE_BETA == 1
    double *beta; // Local beta (soft constraints weights)
    #endif
    #ifdef INITIALIZE_ITERATES
    double *v_ini;
    double *lambda_ini;
    #endif
    double *u_opt; // Local u_opt
    int k; // Local k
    int e_flag; // Local e_flag
    sol_$C_CODE_NAME$ sol = {.update_time = 0.0, .solve_time = 0.0, .polish_time = 0.0, .run_time = 0.0};
    
    // Check inputs and outputs

    // Check number of inputs
    #if SOFT_CONSTRAINTS == 1 && ADAPTIVE_BETA == 1
        #ifdef INITIALIZE_ITERATES
            if(nrhs != 6){
                mexErrMsgIdAndTxt("Spcies:MPCT_ADMM:nrhs:number",
                                  "Six inputs are required");
            }
        #else
            if(nrhs != 4){
                mexErrMsgIdAndTxt("Spcies:MPCT_ADMM:nrhs:number",
                                  "Four inputs are required");
            }
        #endif
    #else
        #ifdef INITIALIZE_ITERATES
            if(nrhs != 5){
                mexErrMsgIdAndTxt("Spcies:MPCT_ADMM:nrhs:number",
                                  "Five inputs are required");
            }
        #else
            if(nrhs != 3){
                mexErrMsgIdAndTxt("Spcies:MPCT_ADMM:nrhs:number",
                                  "Three inputs are required");
            }
        #endif
    #endif

    // Check number of outputs
    if(nlhs == 0){
        mexErrMsgIdAndTxt("Spcies:MPCT_ADMM:nlhs:number",
                          "At least one output required");
    }

    // Check that x0 is of the correct dimension
    if( !mxIsDouble(prhs[0]) || mxGetNumberOfElements(prhs[0]) != nn_ ){
        mexErrMsgIdAndTxt("Spcies:MPCT_ADMM_semiband:nrhs:x0",
                          "x0 must be of dimension %%d", nn_);
    }

    // Check that xr is of the correct dimension
    if( !mxIsDouble(prhs[1]) || mxGetNumberOfElements(prhs[1]) != nn_ ){
        mexErrMsgIdAndTxt("Spcies:MPCT_ADMM_semiband:nrhs:xr",
                          "xr must be of dimension %%d", nn_);
    }

    // Check that ur is of the correct dimension
    if( !mxIsDouble(prhs[2]) || mxGetNumberOfElements(prhs[2]) != mm_ ){
        mexErrMsgIdAndTxt("Spcies:MPCT_ADMM_semiband:nrhs:ur",
                          "ur must be of dimension %%d", mm_);
    }
    
    // Check that beta (if it applies) is of the correct dimension (must be a vector for now)
    #if SOFT_CONSTRAINTS == 1 && ADAPTIVE_BETA == 1
        #ifdef SCALAR_BETA
            if( !mxIsDouble(prhs[3]) || mxGetNumberOfElements(prhs[3]) != 1){
                mexErrMsgIdAndTxt("Spcies:MPCT_ADMM_semiband:nrhs:beta",
                                  "beta must be a scalar. To use a vector, please set option 'adaptive_beta_is_vector' to true");
            }
        #else
            #if CONSTRAINED_OUTPUT == 0
                if( !mxIsDouble(prhs[3]) || mxGetNumberOfElements(prhs[3]) != (NN_+1)*nm_){
                    mexErrMsgIdAndTxt("Spcies:MPCT_ADMM_semiband:nrhs:beta",
                                      "beta must be a vector of dimension %%d. Optionally, a scalar can be used by setting option 'adaptive_beta_is_vector' to false", (NN_+1)*nm_);
                }
            #else
                if( !mxIsDouble(prhs[3]) || mxGetNumberOfElements(prhs[3]) != (NN_+1)*(nm_+pp_)){
                    mexErrMsgIdAndTxt("Spcies:MPCT_ADMM_semiband:nrhs:beta",
                                      "beta must be a vector of dimension %%d. Optionally, a scalar can be used by setting option 'adaptive_beta_is_vector' to false", (NN_+1)*(nm_+pp_));
                }
            #endif
        #endif
        #ifdef INITIALIZE_ITERATES
            #if CONSTRAINED_OUTPUT == 0
                if( !mxIsDouble(prhs[4]) || mxGetNumberOfElements(prhs[4]) != (NN_+1)*nm_){
                    mexErrMsgIdAndTxt("Spcies:MPCT_ADMM_semiband:nrhs:v_ini",
                                      "v_ini must be a vector of dimension %%d", (NN_+1)*nm_);
                }
                if( !mxIsDouble(prhs[5]) || mxGetNumberOfElements(prhs[5]) != (NN_+1)*nm_){
                    mexErrMsgIdAndTxt("Spcies:MPCT_ADMM_semiband:nrhs:lambda_ini",
                                      "lambda_ini must be a vector of dimension %%d", (NN_+1)*nm_);
                }
            #else
                if( !mxIsDouble(prhs[4]) || mxGetNumberOfElements(prhs[4]) != (NN_+1)*(nm_+pp_)){
                    mexErrMsgIdAndTxt("Spcies:MPCT_ADMM_semiband:nrhs:v_ini",
                                      "v_ini must be a vector of dimension %%d", (NN_+1)*(nm_+pp_));
                }
                if( !mxIsDouble(prhs[5]) || mxGetNumberOfElements(prhs[5]) != (NN_+1)*(nm_+pp_)){
                    mexErrMsgIdAndTxt("Spcies:MPCT_ADMM_semiband:nrhs:lambda_ini",
                                      "lambda_ini must be a vector of dimension %%d", (NN_+1)*(nm_+pp_));
                }
            #endif
        #endif
    #else
        #ifdef INITIALIZE_ITERATES
            #if CONSTRAINED_OUTPUT == 0
                if( !mxIsDouble(prhs[3]) || mxGetNumberOfElements(prhs[3]) != (NN_+1)*nm_){
                    mexErrMsgIdAndTxt("Spcies:MPCT_ADMM_semiband:nrhs:v_ini",
                                      "v_ini must be a vector of dimension %%d", (NN_+1)*nm_);
                }
                if( !mxIsDouble(prhs[4]) || mxGetNumberOfElements(prhs[4]) != (NN_+1)*nm_){
                    mexErrMsgIdAndTxt("Spcies:MPCT_ADMM_semiband:nrhs:lambda_ini",
                                      "lambda_ini must be a vector of dimension %%d", (NN_+1)*nm_);
                }
            #else
                if( !mxIsDouble(prhs[3]) || mxGetNumberOfElements(prhs[3]) != (NN_+1)*(nm_+pp_)){
                    mexErrMsgIdAndTxt("Spcies:MPCT_ADMM_semiband:nrhs:v_ini",
                                      "v_ini must be a vector of dimension %%d", (NN_+1)*(nm_+pp_));
                }
                if( !mxIsDouble(prhs[4]) || mxGetNumberOfElements(prhs[4]) != (NN_+1)*(nm_+pp_)){
                    mexErrMsgIdAndTxt("Spcies:MPCT_ADMM_semiband:nrhs:lambda_ini",
                                      "lambda_ini must be a vector of dimension %%d", (NN_+1)*(nm_+pp_));
                }
            #endif
        #endif
    #endif

    // Read input data
    x0 = (double*) mxGetData(prhs[0]);
    xr = (double*) mxGetData(prhs[1]);
    ur = (double*) mxGetData(prhs[2]);
    #if SOFT_CONSTRAINTS == 1 && ADAPTIVE_BETA == 1
    beta = (double*) mxGetData(prhs[3]);
        #ifdef INITIALIZE_ITERATES
        v_ini = (double*) mxGetData(prhs[4]);
        lambda_ini = (double*) mxGetData(prhs[5]);
        #endif
    #else
        #ifdef INITIALIZE_ITERATES
        v_ini = (double*) mxGetData(prhs[3]);
        lambda_ini = (double*) mxGetData(prhs[4]);
        #endif
    #endif
    

    // Prepare output data
    mxArray *z_pt, *v_pt, *lambda_pt, *update_time_pt, *solve_time_pt, *polish_time_pt, *run_time_pt;
    double *z_out, *v_out, *lambda_out, *update_time_out, *solve_time_out, *polish_time_out, *run_time_out, *k_out, *e_flag_out;

    plhs[0] = mxCreateDoubleMatrix(mm_, 1, mxREAL); // u_opt
    plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL); // k
    plhs[2] = mxCreateDoubleMatrix(1, 1, mxREAL); // e_flag

    u_opt = (double*) mxGetData(plhs[0]);
    k_out = (double*) mxGetData(plhs[1]);
    e_flag_out = (double*) mxGetData(plhs[2]);

    // Solution structure
    const char *field_names[] = {"z", "v", "lambda", "update_time", "solve_time", "polish_time", "run_time"};
    plhs[3] = mxCreateStructMatrix(1, 1, 7, field_names);

    z_pt = mxCreateDoubleMatrix((NN_+1)*nm_, 1, mxREAL);
    #if CONSTRAINED_OUTPUT == 0
    v_pt = mxCreateDoubleMatrix((NN_+1)*nm_, 1, mxREAL);
    lambda_pt = mxCreateDoubleMatrix((NN_+1)*nm_, 1, mxREAL);
    #else
    v_pt = mxCreateDoubleMatrix((NN_+1)*(nm_+pp_), 1, mxREAL);
    lambda_pt = mxCreateDoubleMatrix((NN_+1)*(nm_+pp_), 1, mxREAL);
    #endif
    update_time_pt = mxCreateDoubleMatrix(1, 1, mxREAL);
    solve_time_pt = mxCreateDoubleMatrix(1, 1, mxREAL);
    polish_time_pt = mxCreateDoubleMatrix(1, 1, mxREAL);
    run_time_pt = mxCreateDoubleMatrix(1, 1, mxREAL);

    z_out = (double*) mxGetData(z_pt);
    v_out = (double*) mxGetData(v_pt);
    lambda_out = (double*) mxGetData(lambda_pt);
    update_time_out = (double*) mxGetData(update_time_pt);
    solve_time_out = (double*) mxGetData(solve_time_pt);
    polish_time_out = (double*) mxGetData(polish_time_pt);
    run_time_out = (double*) mxGetData(run_time_pt);

    mxSetField(plhs[3], 0, "z", z_pt);
    mxSetField(plhs[3], 0, "v", v_pt);
    mxSetField(plhs[3], 0, "lambda", lambda_pt);
    mxSetField(plhs[3], 0, "update_time", update_time_pt);
    mxSetField(plhs[3], 0, "solve_time", solve_time_pt);
    mxSetField(plhs[3], 0, "polish_time", polish_time_pt);
    mxSetField(plhs[3], 0, "run_time", run_time_pt);

    // Call solver
    #if SOFT_CONSTRAINTS == 1 && ADAPTIVE_BETA == 1
        #ifdef INITIALIZE_ITERATES
        MPCT_ADMM_semiband(x0, xr, ur, beta, v_ini, lambda_ini, u_opt, &k, &e_flag, &sol);
        #else
        MPCT_ADMM_semiband(x0, xr, ur, beta, u_opt, &k, &e_flag, &sol);
        #endif
    #else
        #ifdef INITIALIZE_ITERATES
        MPCT_ADMM_semiband(x0, xr, ur, v_ini, lambda_ini, u_opt, &k, &e_flag, &sol);
        #else
        MPCT_ADMM_semiband(x0, xr, ur, u_opt, &k, &e_flag, &sol);
        #endif
    #endif

    // Set output values
    *k_out = (double) k;
    *e_flag_out = (double) e_flag;
    *update_time_out = sol.update_time;
    *solve_time_out = sol.solve_time;
    *polish_time_out = sol.polish_time;
    *run_time_out = sol.run_time;

    #ifdef DEBUG

    #if CONSTRAINED_OUTPUT == 0
    for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++) {
        z_out[i] = sol.z[i];
        v_out[i] = sol.v[i];
        lambda_out[i] = sol.lambda[i];
    }
    #else
    for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++) {
        z_out[i] = sol.z[i];
    }
    for (unsigned int i = 0 ; i < (NN_+1)*(nm_+pp_) ; i++) {
        v_out[i] = sol.v[i];
        lambda_out[i] = sol.lambda[i];
    }
    #endif

    #endif

}

