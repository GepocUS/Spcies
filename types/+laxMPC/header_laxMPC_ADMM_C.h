#ifndef $INSERT_NAME$_h
#define $INSERT_NAME$_h


$INSERT_DEFINES$


typedef struct {
    double z[NN*nm]; // Optimal z
    double v[NN*nm]; // Optimal v
    double lambda[NN*nm]; // Optimal lambda
    double update_time; // Time taken for the update of the ingredients of the optimization solver
    double solve_time; // Time spent in solving the optimization problem
    double polish_time; // Time taken for extra stuff
    double run_time; // Time taken in the execution of the whole MPC solver function, equal to the sum of all other times
} solution;

#ifdef CONF_MATLAB

#if time_varying == 0
void laxMPC_ADMM(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *pointer_LB, double *pointer_UB, double *u_opt, double *pointer_k, double *e_flag, double *z_opt, double *v_opt, double *lambda_opt, double *update_time, double *solve_time, double *polish_time, double *run_time);
#else
// void laxMPC_ADMM(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *pointer_A, double *pointer_B, double *pointer_Q, double *pointer_R, double *pointer_T, double *u_opt, double *pointer_k, double *e_flag, double *z_opt, double *v_opt, double *lambda_opt);
void laxMPC_ADMM(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *pointer_A, double *pointer_B, double *pointer_Q, double *pointer_R, double *pointer_LB, double *pointer_UB, double *u_opt, double *pointer_k, double *e_flag, double *z_opt, double *v_opt, double *lambda_opt, double *update_time, double *solve_time, double *polish_time, double *run_time);
#endif

#else

#if time_varying == 0
void laxMPC_ADMM(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *pointer_LB, double *pointer_UB, double *u_opt, int *pointer_k, int *e_flag, solution *sol);//, double *update_time, double *solve_time, double *polish_time, double *run_time);
#else
// void laxMPC_ADMM(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *pointer_A, double *pointer_B, double *pointer_Q, double *pointer_R, double *pointer_T, double *u_opt, int *pointer_k, int *e_flag, solution *sol);
void laxMPC_ADMM(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *pointer_A, double *pointer_B, double *pointer_Q, double *pointer_R, double *pointer_LB, double *pointer_UB, double *u_opt, int *pointer_k, int *e_flag, solution *sol);//, double *update_time, double *solve_time, double *polish_time, double *run_time);
#endif

#endif

#endif

// This code is generated by the Spcies toolbox: https://github.com/GepocUS/Spcies

