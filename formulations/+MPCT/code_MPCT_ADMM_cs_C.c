/**
 * Sparse ADMM solver for the MPCT formulation using an extended state space
 *
 * ARGUMENTS:
 * The current system state is given in "x0_in". Pointer to array of size nn_.
 * The state reference is given in "xr_in". Pointer to array of size nn_.
 * The input reference is given in "ur_in". Pointer to array of size mm_.
 * The optimal control action is returned in "u_opt". Pointer to array of size mm_.
 * The number of iterations is returned in "k_in". Pointer to int.
 * The exit flag is returned in "e_flag". Pointer to int.
 *       1: Algorithm converged successfully.
 *      -1: Algorithm did not converge within the maximum number of iterations. Returns current iterate.
 * The optimal decision variables and dual variables are returned in the solution structure sol.
 * Computation times are also returned in the structure sol.
 *
 */

void MPCT_ADMM_cs(double *x0_in, double *xr_in, double *ur_in, double *u_opt, int *k_in, int *e_flag, sol_$INSERT_NAME$ *sol){

#if MEASURE_TIME == 1

#if WIN32
static LARGE_INTEGER start, post_update, post_solve, post_polish;
#else // If Linux
struct timespec start, post_update, post_solve, post_polish;
#endif

read_time(&start);

#endif

// Initialize solver variables
int done = 0; // Flag used to determine when the algorithm should exit
int k = 0; // Number of iterations
double xr[nn_] = {0.0}; // State reference
double ur[mm_] = {0.0}; // Control input reference
double z[NN_*dnm_] = {0.0}; // Decision variable z
double v[NN_*dnm_] = {0.0}; // Decision variable v
double lambda[NN_*dnm_] = {0.0}; // Decision variable lambda
double v1[NN_*dnm_] = {0.0}; // Value of z at the last iteration
double b[nn_] = {0.0}; // First nn_ components of vector b (the rest are known to be zero)
double q[dnm_] = {0.0}; // Cost function vector of P1 (contains xr and ur information)
double q_hat[NN_*dnm_] = {0.0}; // Vector used to compute z
double aux[NN_*dnm_] = {0.0}; // Vector used to compute z
double rhs[nrow_AHi] = {0.0}; // Vector for storing the right-hand-side of the W system of equations
double mu[nrow_AHi] = {0.0}; // Solution of the W system os equations

unsigned int res_flag = 0; // Flag used to determine if the exit condition is satisfied
double res_fixed_point; // Variable used to determine if a fixed point has been reached
double res_primal_feas; // Variable used to determine if primal feasibility is satisfied

// Constant variables
$INSERT_CONSTANTS$

// Obtain variables in scaled units
#if in_engineering == 1
for(unsigned int i = 0; i < nn_; i++){
    b[i] = scaling_x[i]*( x0_in[i] - OpPoint_x[i] );
    xr[i] = scaling_x[i]*( xr_in[i] - OpPoint_x[i] );
}
for(unsigned int i = 0; i < mm_; i++){
    ur[i] = scaling_u[i]*( ur_in[i] - OpPoint_u[i] );
}
#endif
#if in_engineering == 0
for(unsigned int i = 0; i < nn_; i++){
    b[i] = x0_in[i];
    xr[i] = xr_in[i];
}
for(unsigned int i = 0; i < mm_; i++){
    ur[i] = ur_in[i];
}
#endif

// Update vector q with the reference
for(unsigned int j = 0; j < nn_; j++){
    for(unsigned int i = 0; i < nn_; i++){
        q[j+nn_] += Tz[j][i]*xr[i];
    }
}
for(unsigned int j = 0; j < mm_; j++){
    for(unsigned int i = 0; i < mm_; i++){
        q[j+2*nn_+mm_] += Sz[j][i]*ur[i];
    }
}

// Measure time
#if MEASURE_TIME == 1
read_time(&post_update);
get_elapsed_time(&sol->update_time, &post_update, &start);
#endif

// Algorithm
while(done == 0){

    k += 1; // Increment iteration counter

    // Save the value of v into variable v1
    memcpy(v1, v, sizeof(double)*NN_*dnm_);

    //********** Update z **********//

    // Compute q_hat = q + lambda - rho*v
    for(unsigned int j = 0; j < NN_*dnm_; j++){
        #ifdef SCALAR_RHO
        q_hat[j] = q[j %% dnm_] + lambda[j] - rho*v[j];
        #else
        q_hat[j] = q[j %% dnm_] + lambda[j] - rho[j]*v[j];
        #endif
    }

    // Compute the right-hand-side of the W*mu = rhs system of equations
    // We first perform a sparse (CSR) matrix-vector multiplication
    for(unsigned int i = 0; i < nrow_AHi; i++){
        rhs[i] = 0.0;
        for(unsigned int j = AHi_row[i]; j < AHi_row[i+1]; j++){
            rhs[i] += AHi_val[j]*q_hat[AHi_col[j]];            
        }
    }
    for(unsigned int j = 0; j < nn_; j++){
        rhs[j] -= b[j];
    }

    // Solve the W*mu = rhs system of equations
    // We solve it by solving its sparse LDL representation using the method
    // used in the QDLDL solver ( https://github.com/oxfordcontrol/qdldl)
    for(unsigned int j = 0; j < nrow_AHi; j++){
        mu[j] = rhs[j];
    }

    // Forward substitution
    for(unsigned int i = 0; i < nrow_AHi; i++){
        for(unsigned int j = L_col[i]; j < L_col[i+1]; j++){ 
            mu[L_row[j]] -= L_val[j]*mu[i];
        }
    }

    // Divide by the diagonal matrix D
    for(unsigned int j = 0; j < nrow_AHi; j++){
        mu[j] *= Dinv[j];
    }

    // Backward substitution
    for(unsigned int i = nrow_AHi-1; i != -1; i--){
        for(unsigned int j = L_col[i]; j < L_col[i+1]; j++){ 
            mu[i] -= L_val[j]*mu[L_row[j]];
        }
    }

    // Compute z

    // Add the Hi*q_hat term

    for(unsigned int i = 0; i < NN_*dnm_; i++){
            z[i] = 0.0;
        for(unsigned int j = Hi_row[i]; j < Hi_row[i+1]; j++){
            z[i] += Hi_val[j]*q_hat[Hi_col[j]];            
        }
    }

    // Add the sparse matrix-vector multiplication of HiA*mu
    for(unsigned int i = 0; i < nrow_HiA; i++){
        for(unsigned int j = HiA_row[i]; j < HiA_row[i+1]; j++){
            z[i] += HiA_val[j]*mu[HiA_col[j]];            
        }
    }

    //********** Update v **********//

    for(unsigned int j = 0; j < NN_*dnm_; j++){
        #ifdef SCALAR_RHO
        v[j] = z[j] + rho_i*lambda[j];
        #else
        v[j] = z[j] + rho_i[j]*lambda[j];
        #endif
        v[j] = (v[j] > LB[j]) ? v[j] : LB[j]; // maximum between v and the lower bound
        v[j] = (v[j] > UB[j]) ? UB[j] : v[j]; // minimum between v and the upper bound
    }
    
    //********** Update lambda **********//

    for(unsigned int j = 0; j < NN_*dnm_; j++){
        #ifdef SCALAR_RHO
        lambda[j] = lambda[j] + rho*( z[j] - v[j] );
        #else
        lambda[j] = lambda[j] + rho[j]*( z[j] - v[j] );
        #endif
    }

    //********** Compute residuals **********//

    res_flag = 0; // Reset the residual flag

    for(unsigned int j = 0; j < NN_*dnm_; j++){
        res_fixed_point = v1[j] - v[j];
        res_primal_feas = z[j] - v[j];
        // Obtain absolute values
        res_fixed_point = ( res_fixed_point > 0.0 ) ? res_fixed_point : -res_fixed_point;
        res_primal_feas = ( res_primal_feas > 0.0 ) ? res_primal_feas : -res_primal_feas;
        if( res_fixed_point > tol || res_primal_feas > tol){
            res_flag = 1;
            break;
        }
    }

    //********** Exit condition **********//
    
    if(res_flag == 0){
        done = 1;
        *e_flag = 1;
    }
    else if( k >= k_max ){
        done = 1;
        *e_flag = -1;
    }

}

// Measure time
#if MEASURE_TIME == 1
read_time(&post_solve);
get_elapsed_time(&sol->solve_time, &post_solve, &post_update);
#endif

// Control action
#if in_engineering == 1
for(unsigned int j = 0; j < mm_; j++){
    u_opt[j] = v[2*nn_+j]*scaling_i_u[j] + OpPoint_u[j];
}
#endif
#if in_engineering == 0
for(unsigned int j = 0; j < mm_; j++){
    u_opt[j] = v[2*nn_+j];
}
#endif

// Return number of iterations
*k_in = k;

// Save solution into structure
#ifdef DEBUG

for(unsigned int j = 0; j < NN_*dnm_; j++){
    sol->z[j] = z[j];
    sol->v[j] = v[j];
    sol->lambda[j] = lambda[j];
}

#endif

// Measure time
#if MEASURE_TIME == 1
read_time(&post_polish);
get_elapsed_time(&sol->polish_time, &post_polish, &post_solve);
get_elapsed_time(&sol->run_time, &post_polish, &start);
#endif

}

#if MEASURE_TIME == 1

spcies_snippet_get_elapsed_time();

spcies_snippet_read_time();

#endif

