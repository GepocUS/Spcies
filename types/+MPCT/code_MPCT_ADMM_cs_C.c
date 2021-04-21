/**
 * Sparse ADMM solver for the MPCT formulation using an extended state space
 *
 * ARGUMENTS:
 * The current system state is given in "pointer_x0". Pointer to array of size nn.
 * The state reference is given in "pointer_xr". Pointer to array of size nn.
 * The input reference is given in "pointer_ur". Pointer to array of size mm.
 * The optimal control action is returned in "u_opt". Pointer to array of size mm.
 * The number of iterations is returned in "pointer_k". Pointer to int.
 * The exit flag is returned in "e_flag". Pointer to int.
 *       1: Algorithm converged successfully.
 *      -1: Algorithm did not converge within the maximum number of iterations. Returns current iterate.
 * The optimal decision variables and dual variables are returned in the solution structure sol.
 *
 * If CONF_MATLAB is defined, them the solver uses slightly different arguments for the mex file.
 *
 */

#include <stdio.h>

#ifdef CONF_MATLAB

void MPCT_ADMM_cs(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *u_opt, double *pointer_k, double *e_flag, double *z_opt, double *v_opt, double *lambda_opt){

#else

void MPCT_ADMM_cs(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *u_opt, int *pointer_k, int *e_flag, solution_MPCT_ess *sol){

#endif

// Initialize solver variables
int done = 0; // Flag used to determine when the algorithm should exit
#ifdef CONF_MATLAB
double k = 0.0; // Number of iterations. In the Matlab case it is easier if it is defined as a double
#else
int k = 0; // Number of iterations
#endif
double xr[nn] = {0.0}; // State reference
double ur[mm] = {0.0}; // Control input reference
double z[NN*dnm] = {0.0}; // Decision variable z
double v[NN*dnm] = {0.0}; // Decision variable v
double lambda[NN*dnm] = {0.0}; // Decision variable lambda
double z1[NN*dnm] = {0.0}; // Value of z at the last iteration
double b[nn] = {0.0}; // First nn components of vector b (the rest are known to be zero)
double q[dnm] = {0.0}; // Cost function vector of P1 (contains xr and ur information)
double q_hat[NN*dnm] = {0.0}; // Vector used to compute z
double aux[NN*dnm] = {0.0}; // Vector used to compute z
double rhs[nrow_AHi] = {0.0}; // Vector for storing the right-hand-side of the W system of equations
double mu[nrow_AHi] = {0.0}; // Solution of the W system os equations

unsigned int res_flag = 0; // Flag used to determine if the exit condition is satisfied
double res_fixed_point; // Variable used to determine if a fixed point has been reached
double res_primal_feas; // Variable used to determine if primal feasibility is satisfied

// Constant variables
$INSERT_CONSTANTS$

// Obtain variables in scaled units
#if in_engineering == 1
for(unsigned int i = 0; i < nn; i++){
    b[i] = scaling_x[i]*( pointer_x0[i] - OpPoint_x[i] );
    xr[i] = scaling_x[i]*( pointer_xr[i] - OpPoint_x[i] );
}
for(unsigned int i = 0; i < mm; i++){
    ur[i] = scaling_u[i]*( pointer_ur[i] - OpPoint_u[i] );
}
#endif
#if in_engineering == 0
for(unsigned int i = 0; i < nn; i++){
    b[i] = pointer_x0[i];
    xr[i] = pointer_xr[i];
}
for(unsigned int i = 0; i < mm; i++){
    ur[i] = pointer_ur[i];
}
#endif

// Update vector q with the reference
for(unsigned int j = 0; j < nn; j++){
    for(unsigned int i = 0; i < nn; i++){
        q[j+nn] += Tz[j][i]*xr[i];
    }
}
for(unsigned int j = 0; j < mm; j++){
    for(unsigned int i = 0; i < mm; i++){
        q[j+2*nn+mm] += Sz[j][i]*ur[i];
    }
}

// Algorithm
while(done == 0){

    k += 1; // Increment iteration counter

    // Save the value of z into variable z1
    memcpy(z1, z, sizeof(double)*NN*dnm);

    //********** Update z **********//

    // Compute q_hat = q + lambda - rho*v
    for(unsigned int j = 0; j < NN*dnm; j++){
        #ifdef SCALAR_RHO
        q_hat[j] = q[j %% dnm] + lambda[j] - rho*v[j];
        #else
        q_hat[j] = q[j %% dnm] + lambda[j] - rho[j]*v[j];
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
    for(unsigned int j = 0; j < nn; j++){
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

    for(unsigned int i = 0; i < NN*dnm; i++){
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

    for(unsigned int j = 0; j < NN*dnm; j++){
        #ifdef SCALAR_RHO
        v[j] = z[j] + rho_i*lambda[j];
        #else
        v[j] = z[j] + rho_i[j]*lambda[j];
        #endif
        v[j] = (v[j] > LB[j]) ? v[j] : LB[j]; // maximum between v and the lower bound
        v[j] = (v[j] > UB[j]) ? UB[j] : v[j]; // minimum between v and the upper bound
    }
    
    //********** Update lambda **********//

    for(unsigned int j = 0; j < NN*dnm; j++){
        #ifdef SCALAR_RHO
        lambda[j] = lambda[j] + rho*( z[j] - v[j] );
        #else
        lambda[j] = lambda[j] + rho[j]*( z[j] - v[j] );
        #endif
    }

    //********** Compute residuals **********//

    res_flag = 0; // Reset the residual flag

    for(unsigned int j = 0; j < NN*dnm; j++){
        res_fixed_point = z1[j] - z[j];
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
        #ifdef CONF_MATLAB
        e_flag[0] = 1.0;
        #else
        *e_flag = 1;
        #endif
    }
    else if( k >= k_max ){
        done = 1;
        #ifdef CONF_MATLAB
        e_flag[0] = -1.0;
        #else
        *e_flag = -1;
        #endif
    }

}

// Control action
#if in_engineering == 1
for(unsigned int j = 0; j < mm; j++){
    u_opt[j] = v[2*nn+j]*scaling_i_u[j] + OpPoint_u[j];
}
#endif
#if in_engineering == 0
for(unsigned int j = 0; j < mm; j++){
    u_opt[j] = v[2*nn+j];
}
#endif

// Return number of iterations
#ifdef CONF_MATLAB
pointer_k[0] = k;
#else
*pointer_k = k;
#endif

// Save solution into structure
#ifdef DEBUG

#ifdef CONF_MATLAB

for(unsigned int j = 0; j < NN*dnm; j++){
    z_opt[j] = z[j];
    v_opt[j] = v[j];
    lambda_opt[j] = lambda[j];
}

#else

for(unsigned int j = 0; j < NN*dnm; j++){
    sol.z[j] = z[j];
    sol.v[j] = v[j];
    sol.lambda[j] = lambda[j];
}

#endif

#endif

}

