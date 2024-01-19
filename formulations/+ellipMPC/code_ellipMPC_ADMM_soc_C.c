/**
 * Sparse ADMM solver for the MPC formulation subject to terminal ellipsoidal constraint.
 * This version imposes the terminal constraint using a second order cone constraint.
 *
 * ARGUMENTS:
 * The current system state is given in "pointer_x0". Pointer to array of size nn.
 * The state reference is given in "pointer_xr". Pointer to array of size nn.
 * The input reference is given in "pointer_ur". Pointer to array of size mm.
 * The "size" of the terminal constraints "pointer_r". Pointer to a double.
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

#ifdef CONF_MATLAB

void ellipMPC_ADMM_soc(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *r_ellip, double *u_opt, double *pointer_k, double *e_flag, double *z_opt, double *s_opt, double *z_hat_opt, double *s_hat_opt, double *lambda_opt, double *mu_opt){

#else

void ellipMPC_ADMM_soc(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *r_ellip, double *u_opt, int *pointer_k, int *e_flag, solution *sol){

#endif

// Initialize solver variables
int done = 0; // Flag used to determine when the algorithm should exit
#ifdef CONF_MATLAB
double k = 0.0; // Number of iterations. In the Matlab case it is easier if it is defined as a double
#else
int k = 0; // Number of iterations
#endif
double x0[nn]; // Current system state
double xr[nn] = {0.0}; // State reference
double ur[mm] = {0.0}; // Control input reference
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
for(unsigned int i = 0; i < nn; i++){
    x0[i] = scaling_x[i]*( pointer_x0[i] - OpPoint_x[i] );
    xr[i] = scaling_x[i]*( pointer_xr[i] - OpPoint_x[i] );
}
for(unsigned int i = 0; i < mm; i++){
    ur[i] = scaling_u[i]*( pointer_ur[i] - OpPoint_u[i] );
}
#endif
#if in_engineering == 0
for(unsigned int i = 0; i < nn; i++){
    x0[i] = pointer_x0[i];
    xr[i] = pointer_xr[i];
}
for(unsigned int i = 0; i < mm; i++){
    ur[i] = pointer_ur[i];
}
#endif

// Update first nn elements of bh
for(unsigned int j = 0; j < nn; j++){
    bh[j] = 0.0;
    for(unsigned int i = 0; i < nn; i++){
        bh[j] -= A[j][i]*x0[i];
    }
}

// Introduce r into bh
bh[n_eq-1] = *r_ellip;

// Update the last nn elements of bh
for(unsigned int j = 0; j < nn; j++){
    bh[n_eq + 1 + j] = 0.0;
    for(unsigned int i = 0; i < nn; i++){
        bh[n_eq + 1 + j] -= PhiP[j][i]*xr[i];
    }
}

// Update q: First mm elements
for(unsigned int j = 0; j < mm; j++){
    for(unsigned int i = 0; i < mm; i++){
        q[j] += R[j][i]*ur[i];
    }
}

// Update q: All other elements except the last nn
for(unsigned int k = 0; k < NN-1; k++){
    // Reference xr
    for(unsigned int j = 0; j < nn; j++){
        for(unsigned int i = 0; i < nn; i++){
           q[mm+k*nm+j] += Q[j][i]*xr[i];
        }
    }
    // Reference ur
    for(unsigned int j = 0; j < mm; j++){
        for(unsigned int i = 0; i < mm; i++){
           q[nm+k*nm+j] += R[j][i]*ur[i];
        }
    }
}

// Update q: Last nn elements
for(unsigned int j = 0; j < nn; j++){
    for(unsigned int i = 0; i < nn; i++){
        q[mm+(NN-1)*nm+j] += T[j][i]*xr[i];
    }
}


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
    for(unsigned int j = 0; j < dim-nn-1; j++){
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
    u_opt[j] = z[j]*scaling_i_u[j] + OpPoint_u[j];
}
#endif
#if in_engineering == 0
for(unsigned int j = 0; j < mm; j++){
    u_opt[j] = z[j];
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

for(unsigned int j = 0; j < dim; j++){
    z_opt[j] = z[j];
    z_hat_opt[j] = z_hat[j];
    lambda_opt[j] = lambda[j];
}
// for(unsigned int j = 0; j < n_eq+n_s; j++){
    // z_opt[j] = rhs[j];
// }
for(unsigned int j = 0; j < n_s; j++){
    s_opt[j] = s[j];
    s_hat_opt[j] = s_hat[j];
    mu_opt[j] = mu[j];
}

#else

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

#endif

}

