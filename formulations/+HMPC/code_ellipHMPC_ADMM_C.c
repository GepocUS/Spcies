/**
 * ADMM or SADMM solver for the HMPC formulation.
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

#ifdef CONF_MATLAB

void HMPC_ADMM(double *pointer_x0, double *pointer_xre, double *pointer_xrs, double *pointer_xrc, double *pointer_ure, double *pointer_urs, double *pointer_urc, double *u_opt, double *pointer_k, double *e_flag, double *z_opt, double *s_opt, double *lambda_opt){

#else

void HMPC_ADMM(double *pointer_x0, double *pointer_xre, double *pointer_xrs, double *pointer_xrc, double *pointer_ure, double *pointer_urs, double *pointer_urc, double *u_opt, int *pointer_k, int *e_flag, solution *sol){

#endif

// Initialize solver variables
int done = 0; // Flag used to determine when the algorithm should exit
#ifdef CONF_MATLAB
double k = 0.0; // Number of iterations. In the Matlab case it is easier if it is defined as a double
#else
int k = 0; // Number of iterations
#endif
double x0[nn]; // Current system state
double xre[nn] = {0.0}; // State reference
double xrs[nn] = {0.0}; // State reference
double xrc[nn] = {0.0}; // State reference
double ure[mm] = {0.0}; // Control input reference
double urs[mm] = {0.0}; // Control input reference
double urc[mm] = {0.0}; // Control input reference

// For solving the system of equations
double q[dim] = {0.0}; // Cost function vector

double z[dim]; // Vector of primal decision variables z
double s[n_s] = {0.0}; // Vector of primal decision variables s
double s_proj[n_s] = {0.0}; // Vector of primal decision variables s
double q_hat[dim]; // Vector q_hat, which is the first part of rhs
double b[nn] = {0.0}; // Vector of the equality constraints
double Cz[n_s]; // Auxiliary vector for storing C*z and C*z + s

double s_ant[n_s]; // Vector containing the value of primal at iteration k-1
double lambda[n_s] = {0.0}; // Vector of dual variables dual = (lambda, mu)

double s_norm; // Norm of the second-to-the-last elements of s
double s_proj_step; // Variable used to project s onto the SOCs
double *s_cone = &s[n_box]; // Pointer to the first component of s with cone constraints

unsigned int res_flag = 0; // Flag used to determine if the exit condition is satisfied
double res_primal; // Variable used to determine if primal feasibility has been obtained
double res_dual; // Variable used to determine if dual feasibility
// $INSERT_VARIABLES$

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
    xre[i] = pointer_xre[i];
    xrs[i] = pointer_xrs[i];
    xrc[i] = pointer_xrc[i];
}
for(unsigned int i = 0; i < mm; i++){
    ure[i] = pointer_ure[i];
    urs[i] = pointer_urs[i];
    urc[i] = pointer_urc[i];
}
#endif

// Update the elements of rhs corresponding to the current system state
for(unsigned int j = 0; j < nn; j++){
    b[j] = 0.0;
    for(unsigned int i = 0; i < nn; i++){
        b[j] -= A[j][i]*x0[i];
    }
}

// Update the elements of q corresponding to the reference
for(unsigned int j = 0; j < nn; j++){
    for(unsigned int i = 0; i < nn; i++){
        q[(NN-1)*nm+mm+j] -= Te[j][i]*xre[i] + QQ[j][i]*x0[i];
    }
}
for(unsigned int j = 0; j < nn; j++){
    for(unsigned int i = 0; i < nn; i++){
        q[(NN-1)*nm+nn+mm+j] -= Th[j][i]*xrs[i];
    }
}
for(unsigned int j = 0; j < nn; j++){
    for(unsigned int i = 0; i < nn; i++){
        q[(NN-1)*nm+2*nn+mm+j] -= Th[j][i]*xrc[i] + QQ[j][i]*x0[i];
    }
}

for(unsigned int j = 0; j < mm; j++){
    for(unsigned int i = 0; i < mm; i++){
        q[(NN-1)*nm+3*nn+mm+j] -= Se[j][i]*ure[i];
    }
}
for(unsigned int j = 0; j < mm; j++){
    for(unsigned int i = 0; i < mm; i++){
        q[(NN-1)*nm+3*nn+2*mm+j] -= Sh[j][i]*urs[i];
    }
}
for(unsigned int j = 0; j < mm; j++){
    for(unsigned int i = 0; i < mm; i++){
        q[(NN-1)*nm+3*nn+3*mm+j] -= Sh[j][i]*urc[i];
    }
}

// Algorithm
while(done == 0){

    k += 1; // Increment iteration counter

    // Save the value of primal into variable primal_ant
    memcpy(s_ant, s, sizeof(double)*(n_s));

    //********** Update z **********//

    // Compute q_hat = q + C'*(rho*(z - d) + lambda)
    for(unsigned int i = 0; i < n_s; i++){
        #ifdef USE_SOC
        Cz[i] = rho*(s[i] - d[i]) + lambda[i];
        #else
        Cz[i] = rho*s[i] + lambda[i] ;
        #endif
    }
    for(unsigned int i = 0; i < dim; i++){
        q_hat[i] = q[i];
        for(unsigned int j = Ct_row[i]; j < Ct_row[i+1]; j++){
            q_hat[i] += Ct_val[j]*Cz[Ct_col[j]];            
        }
    }

    // Solve the system of equations
    // We have two ways of solving it
    // If NON_SPARSE is defined, then we use aux = M2*bh - M1*q_hat;
    // If it is not defined then we solve the system of equations W*[primal; v]=rhs
    // using sparse LDL representation using the method from the QDLDL solver ( https://github.com/oxfordcontrol/qdldl)

    for(unsigned int i = 0; i < dim; i++){
        z[i] = 0.0;
    }
    for(unsigned int i = 0; i < dim; i++){
        for(unsigned int j = 0; j < nn; j++){
            z[i] += M2[i][j]*b[j];
        }
    }
    for(unsigned int i = 0; i < dim; i++){
        for(unsigned int j = 0; j < dim; j++){
            z[i] += M1[i][j]*q_hat[j];
        }
    }

    //********** Compute C*z **********//

    for(unsigned int i = 0; i < n_s; i++){
        #ifdef USE_SOC
        Cz[i] = -d[i];
        #else
        Cz[i] = 0.0;
        #endif
        for(unsigned int j = C_row[i]; j < C_row[i+1]; j++){
            Cz[i] += C_val[j]*z[C_col[j]];            
        }
    }

    //********** Update dual in symmetric ADMM **********//

    #ifdef IS_SYMMETRIC
    // Update lambda
    for(unsigned int j = 0; j < n_s; j++){
        lambda[j] += alpha_SADMM*rho*(Cz[j] + s[j]);
    }

    #endif

    //********** Update primal **********//

    for(unsigned int j = 0; j < n_s; j++){
        s[j] = -Cz[j] - rho_i*lambda[j];
    }

    // Project onto the box constraints
    for(unsigned int j = 0; j < n_box; j++){
        s[j] = (s[j] > LB[j]) ? s[j] : LB[j]; // maximum between s and the lower bound
        s[j] = (s[j] > UB[j]) ? UB[j] : s[j]; // minimum between s and the upper bound
    }

    // Project onto the SOC
    #ifdef USE_SOC
    for(unsigned int j = 0; j < n_soc; j++){
        proj_SOC3( &s_cone[3*j], 1.0, 0.0);
    }
    #else
    for(unsigned int j = 0; j < n_y; j++){
        proj_SOC3( &s_cone[3*j], 1.0, LBy[j]);
        proj_SOC3( &s_cone[3*j], -1.0, UBy[j]);
    }
    #endif

    //********** Add s to Cz **********//
    for(unsigned int j = 0; j < n_s; j++){
        Cz[j] += s[j];
    }

    //********** Update dual **********//

    #ifdef IS_SYMMETRIC

    for(unsigned int j = 0; j < n_s; j++){
        lambda[j] += alpha_SADMM*rho*Cz[j];
    }

    #else

    for(unsigned int j = 0; j < n_s; j++){
        lambda[j] += rho*Cz[j];
    }

    #endif

    //********** Update residuals **********//

    res_flag = 0; // Reset the residual flag

    for(unsigned int j = 0; j < n_s; j++){
    // for(unsigned int j = dim+n_s-1; j !=-1; j--){
        res_primal = Cz[j];
        res_dual = s[j] - s_ant[j];

        // Obtain absolute values
        res_primal = ( res_primal > 0.0 ) ? res_primal : -res_primal;
        res_dual = ( res_dual > 0.0 ) ? res_dual : -res_dual;
        if( res_primal > tol_p || res_dual > tol_d){
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
}
for(unsigned int j = 0; j < n_s; j++){
    s_opt[j] = s[j];
    lambda_opt[j] = lambda[j];
}

#else

for(unsigned int j = 0; j < dim; j++){
    sol->z[j] = z[j];
}
for(unsigned int j = 0; j < n_s; j++){
    sol->s[j] = s[j];
    sol->lambda[j] = lambda[j];
}

#endif

#endif

}

//********** Projection to SOC **********//
// Returns, in x, the projection of x onto the shifted-SOC given by alpha and d
// The function assumes that the dimension of x is 3, which is the case in the HMPC solver

void proj_SOC3(double *x, double alpha, double d){

    double x_0 = x[0];
    double x_norm = 0.0;
    double x_proj_step;
    double corrected_x_0;

    // Compute x_norm (the norm of the 2nd-to-last elements of x)
    for(unsigned int j = 1; j < 3; j++){
        x_norm += x[j]*x[j];
    }
    x_norm = sqrt(x_norm);

    // Compute auxliary term
    corrected_x_0 = alpha*(x_0 - d);

    // Project onto the shifted-SOC
    if(x_norm <= corrected_x_0){
    } else if(x_norm <= -corrected_x_0){
        x[0] = d; 
        for(unsigned int j = 1; j < 3; j++){
            x[j] = 0.0;
        }
    } else{
        x_proj_step = (corrected_x_0 + x_norm)/(2*x_norm);
        x[0] = x_proj_step*x_norm*alpha + d;
        for(unsigned int j = 1; j < 3; j++){
            x[j] = x_proj_step*x[j];
        }
    }

}

