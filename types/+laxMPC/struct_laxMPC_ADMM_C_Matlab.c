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
    #if time_varying == 1
    double *A;
    double *B;
    double *Q;
    double *R;
//     double *T;
    #endif
    double *LB;
    double *UB;
    double *u_opt; // Local u_opt
    double *k; // Local k
    double *e_flag; // Local e_flag
    double *z_opt; // Local z_opt
    double *v_opt; // Local v_opt
    double *lambda_opt; // Local lambda_opt
    double *update_time_sol; // Local update_time
    double *solve_time_sol; // Local solve_time
    double *polish_time_sol; // Local polish_time
    double *run_time_sol; // Local run_time

    // Check inputs and outputs

    // Check number of inputs
    #if time_varying == 1
    if(nrhs != 9){
        mexErrMsgIdAndTxt("Spcies:laxMPC:nrhs:number",
                          "Not enough inputs");
    }
    #else
    if(nrhs != 5){
        mexErrMsgIdAndTxt("Spcies:laxMPC:nrhs:number",
                          "Not enough inputs");
    }
    #endif

    // Check number of outputs
    if(nlhs == 0){
        mexErrMsgIdAndTxt("Spcies:laxMPC:nlhs:number",
                          "At least one output required");
    }

    // Check that x0 is of the correct dimension
    if( !mxIsDouble(prhs[0]) || mxGetNumberOfElements(prhs[0]) != nn ){
        mexErrMsgIdAndTxt("Spcies:laxMPC:nrhs:x0",
                          "x0 must be of dimension nn");
    }
    // Check that xr is of the correct dimension
    if( !mxIsDouble(prhs[1]) || mxGetNumberOfElements(prhs[1]) != nn ){
        mexErrMsgIdAndTxt("Spcies:laxMPC:nrhs:xr",
                          "xr must be of dimension nn");
    }
    // Check that ur is of the correct dimension
    if( !mxIsDouble(prhs[2]) || mxGetNumberOfElements(prhs[2]) != mm_ ){
        mexErrMsgIdAndTxt("Spcies:laxMPC:nrhs:ur",
                          "ur must be of dimension mm");
    }

    #if time_varying == 1
    // Check that A is of the correct dimension
    if( !mxIsDouble(prhs[3]) || mxGetNumberOfElements(prhs[3]) != nn*nn ){
        mexErrMsgIdAndTxt("Spcies:laxMPC:nrhs:A",
                          "A must be of dimension nn by nn");
    }
    
    // Check that B is of the correct dimension
    if( !mxIsDouble(prhs[4]) || mxGetNumberOfElements(prhs[4]) != nn*mm_ ){
        mexErrMsgIdAndTxt("Spcies:laxMPC:nrhs:B",
                          "B must be of dimension nn by mm");
    }

    // Check that Q is of the correct dimension
    if( !mxIsDouble(prhs[5]) || mxGetNumberOfElements(prhs[5]) != nn ){
        mexErrMsgIdAndTxt("Spcies:laxMPC:nrhs:Q",
                          "Q must be a diagonal vector of nn elements");
    }
    
    // Check that R is of the correct dimension
    if( !mxIsDouble(prhs[6]) || mxGetNumberOfElements(prhs[6]) != mm_ ){
        mexErrMsgIdAndTxt("Spcies:laxMPC:nrhs:R",
                          "R must be a diagonal vector of mm elements");
    }

//     if( !mxIsDouble(prhs[7]) || mxGetNumberOfElements(prhs[7]) != nn*nn ){
//         mexErrMsgIdAndTxt("Spcies:laxMPC:nrhs:T",
//                           "T must be of dimension nn by nn");
//     }

    // Check that LB is of the correct dimension
    if( !mxIsDouble(prhs[7]) || mxGetNumberOfElements(prhs[7]) != nm ){
        mexErrMsgIdAndTxt("Spcies:laxMPC:nrhs:LB",
                          "LB must be of dimension nm");
    }
    
    // Check that UB is of the correct dimension
    if( !mxIsDouble(prhs[8]) || mxGetNumberOfElements(prhs[8]) != nm ){
        mexErrMsgIdAndTxt("Spcies:laxMPC:nrhs:UB",
                          "UB must be of dimension nm");
    }

    #else

    if( !mxIsDouble(prhs[3]) || mxGetNumberOfElements(prhs[3]) != nm ){
        mexErrMsgIdAndTxt("Spcies:laxMPC:nrhs:LB",
                          "LB must be of dimension nm");
    }
    
    // Check that UB is of the correct dimension
    if( !mxIsDouble(prhs[4]) || mxGetNumberOfElements(prhs[4]) != nm ){
        mexErrMsgIdAndTxt("Spcies:laxMPC:nrhs:UB",
                          "UB must be of dimension nm");
    }

    #endif

    

    // Read input data
    #if MX_HAS_INTERLEAVED_COMPLEX
    x0 = mxGetDoubles(prhs[0]);
    #else
    x0 = mxGetPr(prhs[0]);
    #endif

    #if MX_HAS_INTERLEAVED_COMPLEX
    xr = mxGetDoubles(prhs[1]);
    #else
    xr = mxGetPr(prhs[1]);
    #endif

    #if MX_HAS_INTERLEAVED_COMPLEX
    ur = mxGetDoubles(prhs[2]);
    #else
    ur = mxGetPr(prhs[2]);
    #endif

    #if time_varying == 1

    #if MX_HAS_INTERLEAVED_COMPLEX
    A = mxGetDoubles(prhs[3]);
    #else
    A = mxGetPr(prhs[3]);
    #endif

    #if MX_HAS_INTERLEAVED_COMPLEX
    B = mxGetDoubles(prhs[4]);
    #else
    B = mxGetPr(prhs[4]);
    #endif

    #if MX_HAS_INTERLEAVED_COMPLEX
    Q = mxGetDoubles(prhs[5]);
    #else
    Q = mxGetPr(prhs[5]);
    #endif

    #if MX_HAS_INTERLEAVED_COMPLEX
    R = mxGetDoubles(prhs[6]);
    #else
    R = mxGetPr(prhs[6]);
    #endif

//     #if MX_HAS_INTERLEAVED_COMPLEX
//     T = mxGetDoubles(prhs[7]);
//     #else
//     T = mxGetPr(prhs[7]);
//     #endif

    #if MX_HAS_INTERLEAVED_COMPLEX
    LB = mxGetDoubles(prhs[7]);
    #else
    LB = mxGetPr(prhs[7]);
    #endif

    #if MX_HAS_INTERLEAVED_COMPLEX
    UB = mxGetDoubles(prhs[8]);
    #else
    UB = mxGetPr(prhs[8]);
    #endif

    #else

    #if MX_HAS_INTERLEAVED_COMPLEX
    LB = mxGetDoubles(prhs[3]);
    #else
    LB = mxGetPr(prhs[3]);
    #endif

    #if MX_HAS_INTERLEAVED_COMPLEX
    UB = mxGetDoubles(prhs[4]);
    #else
    UB = mxGetPr(prhs[4]);
    #endif


    #endif

    

    // Prepare output data
    plhs[0] = mxCreateDoubleMatrix(mm_, 1, mxREAL); // u_opt
    plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL); // k
    plhs[2] = mxCreateDoubleMatrix(1, 1, mxREAL); // e_flag

    const char *field_names[] = {"z", "v", "lambda", "update_time", "solve_time", "polish_time", "run_time"};
    plhs[3] = mxCreateStructMatrix(1, 1, 7, field_names);

    #if MX_HAS_INTERLEAVED_COMPLEX
    u_opt = mxGetDoubles(plhs[0]);
    #else
    u_opt = mxGetData(plhs[0]);
    #endif

    #if MX_HAS_INTERLEAVED_COMPLEX
    k = mxGetDoubles(plhs[1]);
    #else
    k = mxGetData(plhs[1]);
    #endif

    #if MX_HAS_INTERLEAVED_COMPLEX
    e_flag = mxGetDoubles(plhs[2]);
    #else
    e_flag = mxGetData(plhs[2]);
    #endif

    mxArray *z, *v, *lambda, *update_time, *solve_time, *polish_time, *run_time;
    z = mxCreateDoubleMatrix(NN*nm, 1, mxREAL);
    v = mxCreateDoubleMatrix(NN*nm, 1, mxREAL);
    lambda = mxCreateDoubleMatrix(NN*nm, 1, mxREAL);
    update_time = mxCreateDoubleMatrix(1, 1, mxREAL);
    solve_time = mxCreateDoubleMatrix(1, 1, mxREAL);
    polish_time = mxCreateDoubleMatrix(1, 1, mxREAL);
    run_time = mxCreateDoubleMatrix(1, 1, mxREAL);

    #if MX_HAS_INTERLEAVED_COMPLEX
    z_opt = mxGetDoubles(z);
    #else
    z_opt = mxGetData(z);
    #endif

    #if MX_HAS_INTERLEAVED_COMPLEX
    v_opt = mxGetDoubles(v);
    #else
    v_opt = mxGetData(v);
    #endif

    #if MX_HAS_INTERLEAVED_COMPLEX
    lambda_opt = mxGetDoubles(lambda);
    #else
    lambda_opt = mxGetData(lambda);
    #endif

    #if MX_HAS_INTERLEAVED_COMPLEX
    update_time_sol = mxGetDoubles(update_time);
    #else
    update_time_sol = mxGetData(update_time);
    #endif

    #if MX_HAS_INTERLEAVED_COMPLEX
    solve_time_sol = mxGetDoubles(solve_time);
    #else
    solve_time_sol = mxGetData(solve_time);
    #endif

    #if MX_HAS_INTERLEAVED_COMPLEX
    polish_time_sol = mxGetDoubles(polish_time);
    #else
    polish_time_sol = mxGetData(polish_time);
    #endif

    #if MX_HAS_INTERLEAVED_COMPLEX
    run_time_sol = mxGetDoubles(run_time);
    #else
    run_time_sol = mxGetData(run_time);
    #endif

    mxSetField(plhs[3], 0, "z", z);
    mxSetField(plhs[3], 0, "v", v);
    mxSetField(plhs[3], 0, "lambda", lambda);
    mxSetField(plhs[3], 0, "update_time", update_time);
    mxSetField(plhs[3], 0, "solve_time", solve_time);
    mxSetField(plhs[3], 0, "polish_time", polish_time);
    mxSetField(plhs[3], 0, "run_time", run_time);

    // Call solver
    #if time_varying == 1
//     laxMPC_ADMM(x0, xr, ur, A, B, Q, R, T, u_opt, k, e_flag, z_opt, v_opt, lambda_opt);
    laxMPC_ADMM(x0, xr, ur, A, B, Q, R, LB, UB, u_opt, k, e_flag, z_opt, v_opt, lambda_opt, update_time_sol, solve_time_sol, polish_time_sol, run_time_sol);
    #else
    laxMPC_ADMM(x0, xr, ur, LB, UB, u_opt, k, e_flag, z_opt, v_opt, lambda_opt, update_time_sol, solve_time_sol, polish_time_sol, run_time_sol);
    #endif
}

// This code is generated by the Spcies toolbox: https://github.com/GepocUS/Spcies

