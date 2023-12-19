/**
 * ADMM solver for the MPCT formulation using Woodbury Matrix Identity
 *
 * ARGUMENTS:
 * The current system state is given in "pointer_x0". Pointer to array of size nn_.
 * The state reference is given in "pointer_xr". Pointer to array of size nn_.
 * The input reference is given in "pointer_ur". Pointer to array of size mm_.
 * The optimal constrol action is returned in "u_opt". Pointer to array of size mm_.
 * The number of iterations is returned in "e_flag". Pointer to int.
 *      1: Algorithm converged succesfully.
 *      -1: Algorithm did not converge within the maximum number of iterations. Returns current iterate.
 * The optimal decision variables and dual variables are returned in the solution structure sol.
 * 
 * If CONF_MATLAB is defined, then the solver uses slightly different arguments for the mex file.
 * 
 */

#include <stdio.h>

#ifdef CONF_MATLAB

void MPCT_ADMM_band(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *u_opt, double *pointer_k, double *e_flag, double *z_opt, double *v_opt, double *lambda_opt){

#else

void MPCT_ADMM_band(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *u_opt, double *pointer_k, double *e_flag, solution_MPCT *sol){

#endif

    // Initialize solver variables
    int done = 0;
    #ifdef CONF_MATLAB
    double k = 0.0; // Number of iterations. In the Matlab case it is easier if it is defined as a double
    #else
    int k = 0; // Number of iterations
    #endif
    double x0[nn_] = {0.0}; // Current system state
    double xr[nn_] = {0.0}; // State reference
    double ur[mm_] = {0.0}; // Input reference
    double z[(NN_+1)*nm_] = {0.0}; // Decision variable z
    double v[(NN_+1)*nm_] = {0.0}; // Decision variable v
    double v_old[(NN_+1)*nm_] = {0.0}; // Decision variable v in the previous iteration
    double lambda[(NN_+1)*nm_] = {0.0}; // Decision variable lambda
    double q[(NN_+1)*nm_] = {0.0}; // Linear term vector in the functional
    double b[(NN_+2)*nn_] = {0.0}; // Independent term of the equlity constraint
    double xi[(NN_+1)*nm_] = {0.0}; // Used to solve the equality-constrained QP step
    double mu[(NN_+2)*nn_] = {0.0}; // Used to solve the equality-constrained QP step
    double z1_a[(NN_+1)*nm_] = {0.0}; // Used to solve the equality-constrained QP step --> OK size
    double z2_a[2*nm_] = {0.0}; // Used to solve the equality-constrained QP step --> OK size
    double z3_a[(NN_+1)*nm_] = {0.0}; // Used to solve the equality-constrained QP step --> OK size
    double z1_b[(NN_+2)*nn_] = {0.0}; // Used to solve the equality-constrained QP step --> OK size
    double z2_b[2*nm_] = {0.0}; // Used to solve the equality-constrained QP step --> OK size
    double z3_b[(NN_+2)*nn_] = {0.0}; // Used to solve the equality-constrained QP step --> OK size
    double z1_c[(NN_+1)*nm_] = {0.0}; // Used to solve the equality-constrained QP step --> OK size
    double z2_c[2*nm_] = {0.0}; // Used to solve the equality-constrained QP step --> OK size
    double z3_c[(NN_+1)*nm_] = {0.0}; // Used to solve the equality-constrained QP step --> OK size
    double p[(NN_+1)*nm_] = {0.0}; // Used to solve the equality-constrained QP
    double vec[(NN_+1)*nm_] = {0.0}; // Auxiliary vector used to solve the equality-constrained QP step.
    double vec2[(NN_+2)*nn_] = {0.0}; // Auxiliary vector used to solve the equality-constrained QP step.
    double res_fixed_point; // Variable used to determine if a fixed point has been reached
    double res_primal_feas; // Variable used to determine if primal feasibility is satisfied
    unsigned int res_flag = 0; // Flag used to determine if the exit condition is satisfied

    // Constant variables
    $INSERT_CONSTANTS$

    // Obtain variables in scaled units
    #if in_engineering == 1
    for(unsigned int i = 0; i < nn_; i++){
        x0[i] = scaling_x[i]*( pointer_x0[i] - OpPoint_x[i] );
        xr[i] = scaling_x[i]*( pointer_xr[i] - OpPoint_x[i] );
    }
    for(unsigned int i = 0; i < mm_; i++){
        ur[i] = scaling_u[i]*( pointer_ur[i] - OpPoint_u[i] );
    }
    #endif
    #if in_engineering == 0
    for(unsigned int i = 0; i < nn_; i++){
        x0[i] = pointer_x0[i];
        xr[i] = pointer_xr[i];
    }
    for(unsigned int i = 0; i < mm_; i++){
        ur[i] = pointer_ur[i];
    }
    #endif

    // Compute b

    for(unsigned int i = 0 ; i< nn_ ; i++){
        b[i] = x0[i];
    }


    // Compute q

    for(unsigned int i = 0 ; i < nn_ ; i++){
        
        for(unsigned int j = 0 ; j < nn_ ; j++){

            q[NN_*nm_ + i] -= T[i][j] * xr[j];

        }

    }

    for(unsigned int i = 0 ; i < mm_ ; i++){
        
        for(unsigned int j = 0 ; j < mm_ ; j++){

            q[NN_*nm_ + nn_ + i] -= S[i][j] * ur[j];

        }

    }

    // Algorithm
    while(done == 0){


        k += 1;

        // Save the value of v
        memcpy(v_old, v, sizeof(double)*(NN_+1)*nm_);
        
        // Reset acumulator variables
        memset(z1_a, 0, sizeof(double)*(NN_+1)*nm_);
        memset(z2_a, 0, sizeof(double)*2*nm_);
        memset(z3_a, 0, sizeof(double)*(NN_+1)*nm_);
        memset(z1_b, 0, sizeof(double)*(NN_+2)*nn_);
        memset(z2_b, 0, sizeof(double)*2*nm_);
        memset(z3_b, 0, sizeof(double)*(NN_+2)*nn_);
        memset(z1_c, 0, sizeof(double)*(NN_+1)*nm_);
        memset(z2_c, 0, sizeof(double)*2*nm_);
        memset(z3_c, 0, sizeof(double)*(NN_+1)*nm_);

        //********** Equality-constrained QP solve **********//
        // This problem updates z
        for(unsigned int i = 0; i < (NN_+1)*nm_ ; i++){

            #ifdef SCALAR_RHO
            p[i] = q[i] + lambda[i] - rho * v[i];
            #else
            p[i] = q[i] + lambda[i] - rho[i] * v[i];
            #endif

        }

        // Compute xi from eq. (9a) using Alg. 2 from the article
        solve_banded_diag_sys(Q_rho_i, R_rho_i, S_rho_i, T_rho_i, z1_a, p); // Obtains z1_a

        for (unsigned int i = 0 ; i < 2*nm_ ; i++){ // Obtains z2_a

            for (unsigned int j = 0 ; j < (NN_+1)*nm_ ; j++){

                z2_a[i] += M_hat[i][j] * z1_a[j]; // TODO: Quizás se pueda ahorrar aquí multiplicaciones por 0 gracias a la estructura que tiene M_hat (ver en Matlab)

            }

        }

        memset(vec, 0, sizeof(double)*(NN_+1)*nm_);

        for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){

            for (unsigned int j = 0 ; j < 2*nm_ ; j++){

                vec[i] += U_hat[i][j] * z2_a[j]; // TODO: Esta operación es muy esparsa: Aprovechar la estructura de U_hat (ver en Matlab)

            }
            
        }

        solve_banded_diag_sys(Q_rho_i, R_rho_i, S_rho_i, T_rho_i, z3_a, vec); // Obtains z3_a

        for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){ // Obtains xi, which is the solution of eq. (9a)

            xi[i] = z1_a[i] - z3_a[i];

        }

        // Compute mu from eq. (9b) using Alg. 2 from the article

        memset(vec2, 0, sizeof(double)*(NN_+2)*nn_);

        for (unsigned int i = 0 ; i < nn_ ; i++){

            vec2[i] = -b[i];

        }

        for (unsigned int i = 0 ; i < (NN_+2)*nn_ ; i++){

            for (unsigned int j = 0 ; j < (NN_+1)*nm_ ; j++){

                vec2[i] -= G[i][j] * xi[j];

            }

        }

        solve_banded_Chol(Alpha, Beta, z1_b, vec2); // Obtains z1_b // REVISAR AQUÍ

        for (unsigned int i = 0 ; i < 2*nm_ ; i++){ // Obtains z2_b

            for (unsigned int j = 0 ; j < (NN_+2)*nn_ ; j++){

                z2_b[i] += M_tilde[i][j] * z1_b[j]; // M_tilde is dense, so no structure can be exploited

            }

        }

        memset(vec2, 0, sizeof(double)*(NN_+2)*nn_);

        for (unsigned int i = 0 ; i < (NN_+2)*nn_ ; i++){

            for (unsigned int j = 0 ;  j < 2*nm_ ; j++){

                vec2[i] += U_tilde[i][j] * z2_b[j]; // U_tilde is also practically dense, so no structure will be exploited

            } 

        }

        solve_banded_Chol(Alpha, Beta, z3_b, vec2); // Obtains z3_b

        for (unsigned int i = 0 ; i < (NN_+2)*nn_ ; i++){ // Obtains mu, which is the solution of eq. (9b)

            mu[i] = z1_b[i] - z3_b[i];

        }

        // Compute z^{k+1} from eq. (9c) using ALg. 2 from the article
        memset(vec, 0, sizeof(double)*(NN_+1)*nm_);

        for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){

            for (unsigned int j = 0 ; j < (NN_+2)*nn_ ; j++){

                vec[i] -= G[j][i] * mu[j];

            }

            vec[i] -= p[i];

        }

        solve_banded_diag_sys(Q_rho_i, R_rho_i, S_rho_i, T_rho_i, z1_c, vec); // Obtains z1_c

        for (unsigned int i = 0 ; i < 2*nm_ ; i++){ // Obtains z2_c

            for (unsigned int j = 0 ; j < (NN_+1)*nm_ ; j++){

                z2_c[i] += M_hat[i][j] * z1_c[j];

            }

        }

        memset(vec, 0, sizeof(double)*(NN_+1)*nm_);

        for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){

            for (unsigned int j = 0 ; j < 2*nm_ ; j++){

                vec[i] += U_hat[i][j] * z2_c[j];

            }

        }

        solve_banded_diag_sys(Q_rho_i, R_rho_i, S_rho_i, T_rho_i, z3_c, vec); // Obtains z3_c

        for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){ // Obtains z^{k+1}, which is the solution of eq. (9c)

            z[i] = z1_c[i] - z3_c[i];

        }

        //********** Inequality-constrained QP solve **********//
        // This problem updates v

        for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){

            #ifdef SCALAR_RHO
            v[i] = rho_i * lambda[i] + z[i];
            #else
            v[i] = rho_i[i] * lambda[i] + z[i];
            #endif

            v[i] = (v[i] > LB[i]) ? v[i] : LB[i];

            v[i] = (v[i] < UB[i]) ? v[i] : UB[i];

        }

        //********** Update dual variables **********//
        for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){
            
            #ifdef SCALAR_RHO
            lambda[i] += rho * (z[i] - v[i]);
            #else
            lambda[i] += rho[i] * (z[i] - v[i]);
            #endif

        }

        // Compute the residuals

        res_flag = 0; // Reset the residual flag

        for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){
            
            res_fixed_point = v[i] - v_old[i];
            res_primal_feas = z[i] - v[i];
            // Obtain absolute values
            res_fixed_point = (res_fixed_point > 0.0) ? res_fixed_point : -res_fixed_point;
            res_primal_feas = (res_primal_feas > 0.0) ? res_primal_feas : -res_primal_feas;
            
            if (res_fixed_point > tol || res_primal_feas > tol){

                res_flag = 1;
                break;

            }

        }

        // Exit condition
        if (res_flag == 0){

            done = 1;
            *e_flag = 1;

        }
        else if (k >= k_max){

            done = 1;
            *e_flag = -1;        

        }
        
    }

    // Control action
    #if in_engineering == 1
    for (unsigned int i = nn_ ; i < nm_ ; i++){

        u_opt[i-nn_] = v[i] * scaling_i_u[i-nn_] + OpPoint_u[i-nn_];

    }
    #endif
    #if in_engineering == 0
    for (unsigned int i = nn_ ; i < nm_ ; i++){

        u_opt[i-nn_] = v[i];

    }
    #endif

    // Return number of iterations
    *pointer_k = k;

    // Save solution into structure
    #ifdef DEBUG

    #ifdef CONF_MATLAB

    for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){
        
        z_opt[i] = z[i];
        v_opt[i] = v[i];
        lambda_opt[i] = lambda[i];
    
    }

    #else

    for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){
        
        sol->z[i] = z[i];
        sol->v[i] = v[i];
        sol->lambda[i] = lambda[i];
    
    }

    #endif

    #endif


}



void solve_banded_Chol(const double (*Alpha)[nn_][nn_], const double (*Beta)[nn_][nn_], double *z, double *d){
    
    double sum = 0.0;

    double y[(NN_+2)*nn_] = {0.0};
    
    // Forward substitution

    for (unsigned int k=0 ; k < NN_+2 ; k++){
        
        for (unsigned int i = 0 ; i <= nn_-1 ; i++){

            sum = 0.0;

            for(unsigned int p = 0 ; p+1 <= i ; p++){

                sum += Beta[k][p][i] * y[(k)*nn_+p]; 
            
            }

            if (k>0){
                    
                for(unsigned int p = 0; p < nn_ ; p++){
  
                    sum += Alpha[k-1][p][i] * y[(k-1)*nn_+p];

                }
            
            }
            
            y[(k)*nn_+i] = (d[(k)*nn_+i] - sum) / Beta[k][i][i]; // TODO: Remember this is a division by the diagonal of Beta, but the diagonal of Beta is inverted, so we multiplity

        }

    }

    // Backward substitution

    for (unsigned int k = NN_+2 ; k > 0 ; k--){

        for(unsigned int i = nn_ ; i > 0 ; i--){

            sum = 0.0;

            for(unsigned int p = i+1 ; p <= nn_ ; p++){

                sum += Beta[k-1][i-1][p-1] * z[(k-1)*nn_+(p-1)];

            }
            
            if (k < NN_+2){
                
                for(unsigned int p = 0 ; p < nn_ ; p++){
                
                    sum += Alpha[k-1][i-1][p] * z[(k)*nn_+p];
                
                }
            
            }

            z[(k-1)*nn_+i-1] = (y[(k-1)*nn_+i-1] - sum) / Beta[k-1][i-1][i-1]; // TODO: Remember this is a division by the diagonal of Beta, but the diagonal of Beta is inverted, so we multiplity

        }

    }
    

}

void solve_banded_diag_sys(const double (*Q_rho_i)[nn_], const double (*R_rho_i)[mm_], const double (*S_rho_i)[mm_], const double (*T_rho_i)[nn_], double *z, double *d){

    for (unsigned int i = 0 ; i < NN_*nm_ ; i += nm_){ // Moving in groups of nn+mm components

        for (unsigned int j = 0 ; j < nn_ ; j++){ // Moving inside the groups
            
            for (unsigned int k = 0 ; k < nn_ ; k++){ // Multiplying the rows by the corresponding part of the independent term vector
                
                z[i+j] += Q_rho_i[j][k] * d[i+k];

            }

        }

        for (unsigned int j = nn_ ; j < nm_ ; j++){

            for (unsigned int k = 0 ; k < mm_ ; k++){

                z[i+j] += R_rho_i[j-nn_][k] * d[i+nn_+k];

            }

        }

    }

    for (unsigned int j = 0 ; j < nn_ ; j++){

        for (unsigned int k = 0 ; k < nn_ ; k++){

            z[NN_*nm_ + j] += T_rho_i[j][k] * d[NN_*nm_ + k];

        }
        
    }

    for (unsigned int j = nn_ ; j < nm_ ; j++){

        for (unsigned int k = 0 ; k < mm_ ; k++){

            z[NN_*nm_ + j] += S_rho_i[j-nn_][k] * d[NN_*nm_ + nn_ + k];

        }

    }

}