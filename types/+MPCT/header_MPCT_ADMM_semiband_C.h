#ifndef $INSERT_NAME$_h
#define $INSERT_NAME$_h

$INSERT_DEFINES$

typedef struct {
    double z[(NN_+1)*nm_]; // Optimal z
    double v[(NN_+1)*nm_]; // Optimal v
    double lambda[(NN_+1)*nm_]; // Optimal lambda
    double update_time; // Time taken for the update of the ingredients of the optimization solver
    double solve_time; // Time spent in solving the optimization problem
    double polish_time; // Time taken for extra stuff
    double run_time; // Time taken in the execution of the whole MPC solver function, equal to the sum of all other times
} solution_MPCT;

void MPCT_ADMM_semiband(double *x0_in, double *xr_in, double *ur_in, double *u_opt, int *k_in, int *e_flag, solution_MPCT *sol);

void solve_banded_QRST_sys(const double (*Q_rho_i)[nn_], const double (*R_rho_i)[mm_], const double (*S_rho_i)[mm_], const double (*T_rho_i)[nn_], double *z, double *d);

void solve_banded_Chol(const double (*Alpha)[nn_][nn_], const double (*Beta)[nn_][nn_], double *d); 

#endif
