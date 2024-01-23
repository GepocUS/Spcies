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
    double z[dim]; // Optimal z
    double s[n_s]; // Optimal s
    double z_hat[dim]; // Optimal z_hat
    double s_hat[n_s]; // Optimal s_hat
    double lambda[dim]; // Optimal lambda
    double mu[n_s]; // Optimal mu
    double update_time; // Time taken for the update of the ingredients of the optimization solver
    double solve_time; // Time spent in solving the optimization problem
    double polish_time; // Time taken for extra stuff
    double run_time; // Time taken in the execution of the whole MPC solver function, equal to the sum of all other times
} sol_$INSERT_NAME$;

void ellipMPC_ADMM_soc(double *x0_in, double *xr_in, double *ur_in, double *r_ellip, double *u_opt, int *k_in, int *e_flag, sol_$INSERT_NAME$ *sol);

#if MEASURE_TIME == 1
spcies_snippet_get_elapsed_time();
spcies_snippet_read_time();
#endif

#endif

