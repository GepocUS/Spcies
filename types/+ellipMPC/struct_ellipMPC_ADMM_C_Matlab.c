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
    double *k; // Local k
    double *e_flag; // Local e_flag
    double *z_opt; // Local z_opt
    double *v_opt; // Local v_opt
    double *lambda_opt; // Local lambda_opt

    // Check inputs and outputs

    // Check number of inputs
    if(nrhs != 3){
        mexErrMsgIdAndTxt("Spcies:ellipMPC:nrhs:number",
                          "Not enough inputs");
    }

    // Check number of outputs
    if(nlhs == 0){
        mexErrMsgIdAndTxt("Spcies:ellipMPC:nlhs:number",
                          "At least one output required");
    }

    // Check that x0 is of the correct dimension
    if( !mxIsDouble(prhs[0]) || mxGetNumberOfElements(prhs[0]) != nn ){
        mexErrMsgIdAndTxt("Spcies:ellipMPC:nrhs:x0",
                          "x0 must be of dimension nn");
    }
    // Check that xr is of the correct dimension
    if( !mxIsDouble(prhs[1]) || mxGetNumberOfElements(prhs[1]) != nn ){
        mexErrMsgIdAndTxt("Spcies:ellipMPC:nrhs:xr",
                          "xr must be of dimension nn");
    }
    // Check that ur is of the correct dimension
    if( !mxIsDouble(prhs[2]) || mxGetNumberOfElements(prhs[2]) != mm ){
        mexErrMsgIdAndTxt("Spcies:ellipMPC:nrhs:ur",
                          "ur must be of dimension mm");
    }

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

    // Prepare output data
    plhs[0] = mxCreateDoubleMatrix(mm, 1, mxREAL); // u_opt
    plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL); // k
    plhs[2] = mxCreateDoubleMatrix(1, 1, mxREAL); // e_flag

    const char *field_names[] = {"z", "v", "lambda"};
    plhs[3] = mxCreateStructMatrix(1, 1, 3, field_names);

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

    mxArray *z, *v, *lambda;
    z = mxCreateDoubleMatrix(NN*nm, 1, mxREAL);
    v = mxCreateDoubleMatrix(NN*nm, 1, mxREAL);
    lambda = mxCreateDoubleMatrix(NN*nm, 1, mxREAL);

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

    mxSetField(plhs[3], 0, "z", z);
    mxSetField(plhs[3], 0, "v", v);
    mxSetField(plhs[3], 0, "lambda", lambda);

    // Call solver
    ellipMPC_ADMM(x0, xr, ur, u_opt, k, e_flag, z_opt, v_opt, lambda_opt);

}

// This code is generated by the Spcies toolbox: https://github.com/GepocUS/Spcies

