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
    double z[NN_*nm_]; // Optimal z
    double lambda[NN_*nn_]; // Optimal lambda
    double update_time; // Time taken for the update of the ingredients of the optimization solver
    double solve_time; // Time spent in solving the optimization problem
    double polish_time; // Time taken for extra stuff
    double run_time; // Time taken in the execution of the whole MPC solver function, equal to the sum of all other times
} sol_$INSERT_NAME$;

#if TIME_VARYING  == 1
void laxMPC_FISTA(double *x0_in, double *xr_in, double *ur_in, double *A_in, double *B_in, double *Q_in, double *R_in, double *LB_in, double *UB_in, double *u_opt, int *k_in, int *e_flag, sol_$INSERT_NAME$ *sol);
#else
void laxMPC_FISTA(double *x0_in, double *xr_in, double *ur_in, double *u_opt, int *k_in, int *e_flag, sol_$INSERT_NAME$ *sol);
#endif

void compute_z_lambda_laxMPC_FISTA(double *z_0, double (*z)[nm_], double *z_N, double (*lambda)[nn_], double *q, double *qp, const double *QRi, const double *Ti, const double (*AB)[nm_], const double *LB, const double *UB);

void compute_residual_vector_laxMPC_FISTA(double (*res_vec)[nn_], double *z_0, double (*z)[nm_], double *z_N, double *b, const double (*AB)[nm_]);

#if TIME_VARYING  == 1
void solve_W_matrix_form(double (*mu)[nn_], double (*Alpha)[nn_][nn_], double (*Beta)[nn_][nn_]);
#else
void solve_W_matrix_form(double (*mu)[nn_], const double (*Alpha)[nn_][nn_], const double (*Beta)[nn_][nn_]);
#endif

#if MEASURE_TIME == 1
spcies_snippet_get_elapsed_time();
spcies_snippet_read_time();
#endif

#endif

