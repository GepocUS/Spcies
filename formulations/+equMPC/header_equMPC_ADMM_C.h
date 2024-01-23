#ifndef $INSERT_NAME$_h
#define $INSERT_NAME$_h

$INSERT_DEFINES$

#if MEASURE_TIME == 1
#if WIN32
#include <Windows.h>
#else // If Linux
#include <time.h>
#endif
#endif

typedef struct {
    double z[NN_*nm_-nn_]; // Optimal z
    double v[NN_*nm_-nn_]; // Optimal v
    double lambda[NN_*nm_-nn_]; // Optimal lambda
    double update_time; // Time taken for the update of the ingredients of the optimization solver
    double solve_time; // Time spent in solving the optimization problem
    double polish_time; // Time taken for extra stuff
    double run_time; // Time taken in the execution of the whole MPC solver function, equal to the sum of all other times
} sol_$INSERT_NAME$;

#if TIME_VARYING == 1
void equMPC_ADMM(double *x0_in, double *xr_in, double *ur_in, double *A_in, double *B_in, double *Q_in, double *R_in, double *LB_in, double *UB_in, double *u_opt, int *k_in, int *e_flag, sol_$INSERT_NAME$ *sol);
#else
void equMPC_ADMM(double *x0_in, double *xr_in, double *ur_in, double *u_opt, int *k_in, int *e_flag, sol_$INSERT_NAME$ *sol);
#endif

#if MEASURE_TIME == 1
spcies_snippet_get_elapsed_time();
spcies_snippet_read_time();
#endif

#endif

