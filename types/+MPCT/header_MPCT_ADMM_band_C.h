#ifndef $INSERT_NAME$_h
#define $INSERT_NAME$_h

$INSERT_DEFINES$

typedef struct {
    double z[(NN_+1)*nm_]; // Optimal z
    double v[(NN_+1)*nm_]; // Optimal v
    double lambda[(NN_+1)*nm_]; // Optimal lambda
} solution_MPCT;

#ifdef CONF_MATLAB

void MPCT_ADMM_band(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *u_opt, double *pointer_k, double *e_flag, double *z_opt, double *v_opt, double *lambda_opt);

#else

void MPCT_ADMM_band(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *u_opt, double *pointer_k, double *e_flag, solution_MPCT *sol);

#endif

void solve_banded_diag_sys(const double (*Q_rho_i)[nn_], const double (*R_rho_i)[mm_], const double (*S_rho_i)[mm_], const double (*T_rho_i)[nn_], double *z, double *d);

void solve_banded_Chol(const double (*Alpha)[nn_][nn_], const double (*Beta)[nn_][nn_], double *z, double *d); 

#endif
