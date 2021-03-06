#ifndef $INSERT_NAME$_h
#define $INSERT_NAME$_h

$INSERT_DEFINES$

typedef struct {
    double z1[(NN+1)*nm]; // Optimal z
    double z2[nm]; // Optimal v
    double z3[(NN+1)*nm];
    double lambda[(NN+2)*nm + nn]; // Optimal lambda
} solution_MPCT;

#ifdef CONF_MATLAB

void MPCT_EADMM(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *u_opt, double *pointer_k, double *e_flag, double *z1_opt, double *z2_opt, double *z3_opt, double *lambda_opt);

#else

void MPCT_EADMM(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *u_opt, int *pointer_k, int *e_flag, solution_MPCT *sol);

#endif

#endif

// This code is generated by the Spcies toolbox: https://github.com/GepocUS/Spcies

