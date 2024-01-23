/**
 * Sparse ADMM solver for the MPC formulation subject to terminal ellipsoidal constraint.
 * This version imposes the terminal constraint using a second order cone constraint.
 *
 * ARGUMENTS:
 * The current system state is given in "x0_in". Pointer to array of size nn_.
 * The state reference is given in "xr_in". Pointer to array of size nn_.
 * The input reference is given in "ur_in". Pointer to array of size mm_.
 * The "size" of the terminal constraints "pointer_r". Pointer to a double.
 * The optimal control action is returned in "u_opt". Pointer to array of size mm_.
 * The number of iterations is returned in "k_in". Pointer to int.
 * The exit flag is returned in "e_flag". Pointer to int.
 *       1: Algorithm converged successfully.
 *      -1: Algorithm did not converge within the maximum number of iterations. Returns current iterate.
 * The optimal decision variables and dual variables are returned in the solution structure sol.
 * Computation times are also returned in the structure sol.
 *
 */

void ellipMPC_ADMM_soc(double *x0_in, double *xr_in, double *ur_in, double *r_ellip, double *u_opt, int *k_in, int *e_flag, sol_$INSERT_NAME$ *sol){

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
double x0[nn_]; // Current system state
double xr[nn_] = {0.0}; // State reference
double ur[mm_] = {0.0}; // Control input reference
double primal[dim+n_s] = {0.0}; // Vector of primal decision variables primal = (z, s)
double primal_ant[dim+n_s] = {0.0}; // Vector containing the value of primal at iteration k-1
double primal_hat[dim+n_s] = {0.0}; // Vector of primal decision variables primal_hat = (z_hat, s_hat)
double dual[dim+n_s] = {0.0}; // Vector of dual variables dual = (lambda, mu)
double *z = &primal[0]; // Primal decision variable z
double *s = &primal[dim]; // Primal decision variable s
double *z_hat = &primal_hat[0]; // Primal decision variable z_hat
double *s_hat = &primal_hat[dim]; // Primal decision variable s_hat
double *lambda = &dual[0]; // Dual variable lambda
double *mu = &dual[dim]; // Dual variable mu
double s_norm; // Norm of the second-to-the-last elements of s
double s_proj_step; // Variable used to project s onto the SOC
double bh[n_eq+n_s] = {0.0}; // r.h.s. of equality constraints
double q_hat[dim+n_s] = {0.0}; // Vector used to compute primal_hat
double q[dim] = {0.0}; // Cost function vector
double rhs[n_eq+n_s] = {0.0}; // Vector for storing the right-hand-side of the W system of equations
unsigned int res_flag = 0; // Flag used to determine if the exit condition is satisfied
double res_fixed_point; // Variable used to determine if a fixed point has been reached
double res_primal_feas; // Variable used to determine if primal feasibility is satisfied

// Constant variables
$INSERT_CONSTANTS$

// Obtain variables in scaled units
#if in_engineering == 1
for(unsigned int i = 0; i < nn_; i++){
    x0[i] = scaling_x[i]*( x0_in[i] - OpPoint_x[i] );
    xr[i] = scaling_x[i]*( xr_in[i] - OpPoint_x[i] );
}
for(unsigned int i = 0; i < mm_; i++){
    ur[i] = scaling_u[i]*( ur_in[i] - OpPoint_u[i] );
}
#endif
#if in_engineering == 0
for(unsigned int i = 0; i < nn_; i++){
    x0[i] = x0_in[i];
    xr[i] = xr_in[i];
}
for(unsigned int i = 0; i < mm_; i++){
    ur[i] = ur_in[i];
}
#endif

// Update first nn_ elements of bh
for(unsigned int j = 0; j < nn_; j++){
    bh[j] = 0.0;
    for(unsigned int i = 0; i < nn_; i++){
        bh[j] -= A[j][i]*x0[i];
    }
}

// Introduce r into bh
bh[n_eq-1] = *r_ellip;

// Update the last nn_ elements of bh
for(unsigned int j = 0; j < nn_; j++){
    bh[n_eq + 1 + j] = 0.0;
    for(unsigned int i = 0; i < nn_; i++){
        bh[n_eq + 1 + j] -= PhiP[j][i]*xr[i];
    }
}

// Update q: First mm_ elements
for(unsigned int j = 0; j < mm_; j++){
    for(unsigned int i = 0; i < mm_; i++){
        q[j] += R[j][i]*ur[i];
    }
}

// Update q: All other elements except the last nn_
for(unsigned int k = 0; k < NN_-1; k++){
    // Reference xr
    for(unsigned int j = 0; j < nn_; j++){
        for(unsigned int i = 0; i < nn_; i++){
           q[mm_+k*nm_+j] += Q[j][i]*xr[i];
        }
    }
    // Reference ur
    for(unsigned int j = 0; j < mm_; j++){
        for(unsigned int i = 0; i < mm_; i++){
           q[nm_+k*nm_+j] += R[j][i]*ur[i];
        }
    }
}

// Update q: Last nn_ elements
for(unsigned int j = 0; j < nn_; j++){
    for(unsigned int i = 0; i < nn_; i++){
        q[mm_+(NN_-1)*nm_+j] += T[j][i]*xr[i];
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

    // Save the value of primal into variable primal_ant
    memcpy(primal_ant, primal, sizeof(double)*(dim+n_s));

    //********** Update primal_hat **********//

    // Compute q_hat = [q + lambda - sigma*z; mu - rho*s]
    for(unsigned int j = 0; j < dim; j++){
        q_hat[j] = q[j] + lambda[j] - sigma*z[j];
    }
    for(unsigned int j = 0; j < n_s; j++){
        q_hat[j+dim] = mu[j] - rho*s[j];
    }

    // Compute the r.h.s. of the system of equations
    for(unsigned int i = 0; i < nrow_GhHhi; i++){
        rhs[i] = 0.0;
        for(unsigned int j = GhHhi_row[i]; j < GhHhi_row[i+1]; j++){
            rhs[i] += GhHhi_val[j]*q_hat[GhHhi_col[j]];            
        }
    }
    for(unsigned int j = 0; j < n_eq+n_s; j++){
        rhs[j] -= bh[j];
    }

    // Solve the W*mu = rhs system of equations
    // We solve it by solving its sparse LDL representation using the method
    // used in the QDLDL solver ( https://github.com/oxfordcontrol/qdldl)

    // Forward substitution
    for(unsigned int i = 0; i < nrow_GhHhi; i++){
        for(unsigned int j = L_col[i]; j < L_col[i+1]; j++){ 
            rhs[L_row[j]] -= L_val[j]*rhs[i];
        }
    }

    // Divide by the diagonal matrix D
    for(unsigned int j = 0; j < nrow_GhHhi; j++){
        rhs[j] *= Dinv[j];
    }

    // Backward substitution
    for(unsigned int i = nrow_GhHhi-1; i != -1; i--){
        for(unsigned int j = L_col[i]; j < L_col[i+1]; j++){ 
            rhs[i] -= L_val[j]*rhs[L_row[j]];
        }
    }

    // Compute primal_hat

    // Add the Hhi*q_hat term
    for(unsigned int i = 0; i < nrow_Hhi; i++){
            primal_hat[i] = 0.0;
        for(unsigned int j = Hhi_row[i]; j < Hhi_row[i+1]; j++){
            primal_hat[i] += Hhi_val[j]*q_hat[Hhi_col[j]];            
        }
    }

    // Add the sparse matrix-vector multiplication of HhiGhi*mu
    for(unsigned int i = 0; i < nrow_HhiGh; i++){
        for(unsigned int j = HhiGh_row[i]; j < HhiGh_row[i+1]; j++){
            primal_hat[i] += HhiGh_val[j]*rhs[HhiGh_col[j]];            
        }
    }

    //********** Update primal **********//

    // Compute z
    for(unsigned int j = 0; j < dim; j++){
        z[j] = z_hat[j] + sigma_i*lambda[j];
    }
    // Upper and lower bounds
    for(unsigned int j = 0; j < dim-nn_-1; j++){
        z[j] = (z[j] > LB[j]) ? z[j] : LB[j]; // maximum between v and the lower bound
        z[j] = (z[j] > UB[j]) ? UB[j] : z[j]; // minimum between v and the upper bound
    }

    // Compute s
    for(unsigned int j = 0; j < n_s; j++){
        s[j] = s_hat[j] + rho_i*mu[j];
    }
    //Compute norm of the second-to-the-last elements of s
    s_norm = 0.0;
    for(unsigned int j = 1; j < n_s; j++){
        s_norm += s[j]*s[j];
    }
    s_norm = sqrt(s_norm);

    // Project s onto the SOC
    if(s_norm <= s[0]){
    } else if(s_norm <= -s[0]){
        for(unsigned int j = 0; j < n_s; j++){
            s[j] = 0.0;
        }
    } else {
        s_proj_step = (s[0] + s_norm)/(2*s_norm);
        s[0] = s_proj_step*s_norm;
        for(unsigned int j = 1; j < n_s; j++){
            s[j] = s_proj_step*s[j];
        }
    }
    
    //********** Update dual **********//

    // Update lambda
    for(unsigned int j = 0; j < dim; j++){
        lambda[j] += sigma*(z_hat[j] - z[j]);
    }

    // Update mu
    for(unsigned int j = 0; j < n_s; j++){
        mu[j] += rho*(s_hat[j] - s[j]);
    }

    //********** Update residuals **********//

    res_flag = 0; // Reset the residual flag

    for(unsigned int j = 0; j < dim + n_s; j++){
        res_fixed_point = primal_ant[j] - primal[j];
        res_primal_feas = primal[j] - primal_hat[j];
        // Obtain absolute values
        res_fixed_point = ( res_fixed_point > 0.0 ) ? res_fixed_point : -res_fixed_point;
        res_primal_feas = ( res_primal_feas > 0.0 ) ? res_primal_feas : -res_primal_feas;
        if( res_fixed_point > tol_d || res_primal_feas > tol_p){
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
    u_opt[j] = z[j]*scaling_i_u[j] + OpPoint_u[j];
}
#endif
#if in_engineering == 0
for(unsigned int j = 0; j < mm_; j++){
    u_opt[j] = z[j];
}
#endif

// Return number of iterations
*k_in = k;

// Save solution into structure
#ifdef DEBUG

for(unsigned int j = 0; j < dim; j++){
    sol->z[j] = z[j];
    sol->z_hat[j] = z_hat[j];
    sol->lambda[j] = lambda[j];
}
for(unsigned int j = 0; j < n_s; j++){
    sol->s[j] = s[j];
    sol->s_hat[j] = s_hat[j];
    sol->mu[j] = mu[j];
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

