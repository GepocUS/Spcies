/**
 * Sparse FISTA solver for the lax MPC formulation
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

// Constant variables
$INSERT_CONSTANTS$

#ifdef CONF_MATLAB

#if time_varying == 0
void laxMPC_FISTA(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *u_opt, double *pointer_k, double *e_flag, double *z_opt, double *lambda_opt, double *update_time, double *solve_time, double *polish_time, double *run_time){
#else
void laxMPC_FISTA(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *pointer_A, double *pointer_B, double *pointer_Q, double *pointer_R, double *u_opt, double *pointer_k, double *e_flag, double *z_opt, double *lambda_opt, , double *update_time, double *solve_time, double *polish_time, double *run_time){

#else

#if time_varying == 0
void laxMPC_FISTA(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *u_opt, int *pointer_k, int *e_flag, sol_laxMPC_FISTA *sol){
#else
void laxMPC_FISTA(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *pointer_A, double *pointer_B, double *pointer_Q, double *pointer_R, double *u_opt, int *pointer_k, int *e_flag, sol_laxMPC_FISTA *sol){
#endif

#endif

    // Initialize FISTA variables
    int done = 0; // Flag used to determine when the algorithm should exit
    e_flag[0] = 0.0;
    #ifdef CONF_MATLAB
    double k = 0.0; // Number of iterations. In the Matlab case it is easier if it is defined as a double
    #else
    int k = 0; // Number of iterations
    #endif
    double x0[nn] = {0.0}; // Current system state
    double xr[nn] = {0.0}; // State reference
    double ur[mm] = {0.0}; // Control input reference
    #if time_varying == 1
        double A[nn][nn];
        double B[nn][mm_];
        double AB[nn][nm];
        double Q[nn];
        double R[mm_];
        double QRi[nm];
        double Ri[mm_] = {0.0}; // 1./(diag(R))
        double Qi[nn] = {0.0}; // 1./(diag(Q))
        double Alpha[NN-1][nn][nn] = {{{0.0}}};
        double Beta[NN][nn][nn] = {{{0.0}}};
        double inv_Beta[nn][nn] = {{0.0}}; // Inverse of only the current beta is stored
    #endif
    double z[NN-1][nm] = {{0.0}}; // Primal decision variables
    double z_0[mm] = {0.0};
    double z_N[nn] = {0.0};
    double y[NN][nn] = {{0.0}}; // Linearization point
    double lambda[NN][nn] = {{0.0}}; // Dual variables
    double lambda1[NN][nn] = {{0.0}}; // Dual variables in the previous iteration
    double d_lambda[NN][nn] = {{0.0}}; // Dual variables
    double t = 1.0; // Step of the FISTA algorithm
    double t1 = 1.0; // Step of the previous iteration
    double b[nn] = {0.0}; // First nn components of vector b (the rest are known to be zero)
    double q[nm] = {0.0}; // For storing the components of q related to Q and R
    double qT[nn] = {0.0}; // For storing the components of q related to T
    double res = 0.0; // For storing the absolute value of each element of the residual vector
    unsigned int res_flag = 0; // Flag used to determine if the exit condition is satisfied

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
 
    // Update first nn elements of beq
    for(unsigned int j = 0; j < nn; j++){
        b[j] = 0.0;
        for(unsigned int i = 0; i < nn; i++){
            b[j] = b[j] - AB[j][i]*x0[i];
        }
    }

    // Update the reference
    for(unsigned int j = 0; j < nn; j++){
        q[j] = Q[j]*xr[j];
        qT[j] = T[j]*xr[j]; // Revisar si esto hay que dejarlo así, o si necesitamos T completa (matriz diagonal)
    }
    for(unsigned int j = 0; j < mm; j++){
        q[j+nn] = R[j]*ur[j];
    }

    // Initial steps 

    // Compute z_lambda_0
    compute_z_lambda_laxMPC_FISTA(z_0, z, z_N, lambda, q, qT);

    // Compute the residual: We save it into d_lambda to save memory and future computations
    compute_residual_vector_laxMPC_FISTA(d_lambda, z_0, z, z_N, b);

    // Compute delta_lambda_0
    solve_W_matrix_form(d_lambda);

    // Update lambda_0
    for(unsigned int l = 0; l < NN; l++){
        for(unsigned int j = 0; j < nn; j++){
            lambda[l][j] = lambda[l][j] + d_lambda[l][j];
        }
    }

    // Update y_0
    for(unsigned int l = 0; l < NN; l++){
        for(unsigned int j = 0; j < nn; j++){
            y[l][j] = lambda[l][j];
        }
    }

    // Algorithm
    while(done == 0){

        k += 1; // Increment iteration counter

        // Step 0: Save the value of lambda into variable lambda1 and t into t1
        memcpy(lambda1, lambda, sizeof(double)*NN*nn);
        t1 = t;

        // Compute z_lambda_k
        compute_z_lambda_laxMPC_FISTA(z_0, z, z_N, y, q, qT);

        // Compute the residual: We save it into d_lambda to save memory and future computations
        compute_residual_vector_laxMPC_FISTA(d_lambda, z_0, z, z_N, b);

        // Check exit condition
        res_flag = 0; // Reset exit flag

        for(unsigned int l = 0; l < NN; l++){
            for(unsigned int j = 0; j < nn; j++){
                res = d_lambda[l][j];
                res = (res > 0.0) ? res : -res; // Absolute value
                if( res > tol ){
                    res_flag = 1;
                    break;
                }
            }
            if( res_flag == 1){
                break;
            }
        }

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

        // The rest of the steps are unnecessary if done == 1

        if( done == 0 ){

            // Compute delta_lambda_k
            solve_W_matrix_form(d_lambda);

            // Update lambda_k
            for(unsigned int l = 0; l < NN; l++){
                for(unsigned int j = 0; j < nn; j++){
                    lambda[l][j] = y[l][j] + d_lambda[l][j];
                }
            }

            // Update t_k
            t = 0.5*( 1 + sqrt( 1 + 4*t1*t1) );

            // Update y_k
            for(unsigned int l = 0; l < NN; l++){
                for(unsigned int j = 0; j < nn; j++){
                    y[l][j] = lambda[l][j] + (t1 - 1)*(lambda[l][j] - lambda1[l][j])/t;
                }
            }

        }

    }

    // Control action
    #if in_engineering == 1
    for(unsigned int j = 0; j < mm; j++){
        u_opt[j] = z_0[j]*scaling_i_u[j] + OpPoint_u[j];
    }
    #endif
    #if in_engineering == 0
    for(unsigned int j = 0; j < mm; j++){
        u_opt[j] = z_0[j];
    }
    #endif

    // Return number of iterations
    #ifdef CONF_MATLAB
    pointer_k[0] = k; // Number of iterations
    #else
    *pointer_k = k; // Number of iterations
    #endif

    // Save solution into structure
    #ifdef DEBUG

    #ifdef CONF_MATLAB

    // Return z_opt

    // First mm variables
    int count = -1;
    for(unsigned int j = 0; j < mm; j++){
        count++;
        z_opt[count] = z_0[j];
    }

    // All other elements except the last nn
    for(unsigned int l = 0; l < NN-1; l++){
        for(unsigned int j = 0; j < nm; j++){
            count++;
            z_opt[count] = z[l][j];
        }
    }

    // Last nn elements
    for(unsigned int j = 0; j < nn; j++){
        count++;
        z_opt[count] = z_N[j];
    }

    // Return lambda_opt
    count = -1;
    for(unsigned int l = 0; l < NN; l++){
        for(unsigned int j = 0; j < nn; j++){
            count++;
            lambda_opt[count] = y[l][j];
        }
    }

    #else
    // Return z_opt

    // First mm variables
    int count = -1;
    for(unsigned int j = 0; j < mm; j++){
        count++;
        sol->z[count] = z_0[j];
    }

    // All other elements except the last nn
    for(unsigned int l = 0; l < NN-1; l++){
        for(unsigned int j = 0; j < nm; j++){
            count++;
            sol->z[count] = z[l][j];
        }
    }

    // Last nn elements
    for(unsigned int j = 0; j < nn; j++){
        count++;
        sol->z[count] = z_N[j];
    }

    // Return lambda_opt
    count = -1;
    for(unsigned int l = 0; l < NN; l++){
        for(unsigned int j = 0; j < nn; j++){
            count++;
            sol->lambda[count] = y[l][j];
        }
    }

    #endif

    #endif

}

/* compute_z_lambda_laxMPC_FISTA()
*
*/

void compute_z_lambda_laxMPC_FISTA(double *z_0, double z[][nm], double *z_N, double lambda[][nn], double *q, double *qT){

    // Compute first mm elements
    for(unsigned int j = 0; j < mm; j++){

        // Compute the q - G'*lambda part
        z_0[j] = q[j+nn];
        for(unsigned int i = 0; i < nn; i++){
            z_0[j] = z_0[j] - AB[i][j+nn]*lambda[0][i];
        }

        // Compute the solution of the QP
        z_0[j] = z_0[j]*QRi[j+nn]; // Multiply by the inverse of the Hessian
        #ifdef VAR_BOUNDS
        z_0[j] = (z_0[j] > LB0[j]) ? z_0[j] : LB0[j]; // maximum between v and the lower bound
        z_0[j] = (z_0[j] > UB0[j]) ? UB0[j] : z_0[j]; // minimum between v and the upper bound
        #else
        z_0[j] = (z_0[j] > LB[j+nn]) ? z_0[j] : LB[j+nn]; // maximum between v and the lower bound
        z_0[j] = (z_0[j] > UB[j+nn]) ? UB[j+nn] : z_0[j]; // minimum between v and the upper bound
        #endif
    }

    // Compute all the other elements except the last nn
    for(unsigned int l = 0; l < NN-1; l++){

        // Compute the q - G'*lambda part
        for(unsigned int j = 0; j < nm; j++){
            z[l][j] = q[j];
            for(unsigned int i = 0; i < nn; i++){
                z[l][j] = z[l][j] - AB[i][j]*lambda[l+1][i];
            }
        }
        for(unsigned int j = 0; j < nn; j++){
            z[l][j] = z[l][j] + lambda[l][j];
        }

        // Compute the solution of the QP
        for(unsigned int j = 0; j < nm; j++){
            z[l][j] = z[l][j]*QRi[j]; // Multiply by the inverse of the Hessian
            #ifdef VAR_BOUNDS
            z[l][j] = (z[l][j] > LB[l][j]) ? z[l][j] : LB[l][j]; // maximum between v and the lower bound
            z[l][j] = (z[l][j] > UB[l][j]) ? UB[l][j] : z[l][j]; // minimum between v and the upper bound
            #else
            z[l][j] = (z[l][j] > LB[j]) ? z[l][j] : LB[j]; // maximum between v and the lower bound
            z[l][j] = (z[l][j] > UB[j]) ? UB[j] : z[l][j]; // minimum between v and the upper bound
            #endif
        }

    }

    // Compute the last nn elements
    for(unsigned int j = 0; j < nn; j++){

        // Compute the q - G'*lambda part
        z_N[j] = qT[j] + lambda[NN-1][j];

        // Compute the solution of the QP
        z_N[j] = z_N[j]*Ti[j]; // Multiply by the inverse of the Hessian
        #ifdef VAR_BOUNDS
        z_N[j] = (z_N[j] > LBN[j]) ? z_N[j] : LBN[j]; // maximum between v and the lower bound
        z_N[j] = (z_N[j] > UBN[j]) ? UBN[j] : z_N[j]; // minimum between v and the upper bound
        #else
        z_N[j] = (z_N[j] > LB[j]) ? z_N[j] : LB[j]; // maximum between v and the lower bound
        z_N[j] = (z_N[j] > UB[j]) ? UB[j] : z_N[j]; // minimum between v and the upper bound
        #endif

    }

}

/* compute_residual_laxMPC_FISTA
*
*/

void compute_residual_vector_laxMPC_FISTA(double res_vec[][nn], double *z_0, double z[][nm], double *z_N, double *b){

    // Compute the first nn elements
    for(unsigned int j = 0; j < nn; j++){
            res_vec[0][j] = b[j] + z[0][j];
        for(unsigned int i = 0; i < mm; i++){
            res_vec[0][j] = res_vec[0][j] - AB[j][i+nn]*z_0[i];
        }
    }

    // Compute all the other elements except the last nn
    for(unsigned int l = 1; l < NN-1; l++){
        for(unsigned int j = 0; j < nn; j++){
            res_vec[l][j] = z[l][j];
            for(unsigned int i = 0; i < nm; i++){
                res_vec[l][j] = res_vec[l][j] - AB[j][i]*z[l-1][i];
            }
        }
    }

    // Compute the last nn elements
    for(unsigned int j = 0; j < nn; j++){
        res_vec[NN-1][j] = z_N[j];
        for(unsigned int i = 0; i < nm; i++){
            res_vec[NN-1][j] = res_vec[NN-1][j] - AB[j][i]*z[NN-2][i];
        }
    }

}

void solve_W_matrix_form(double mu[][nn]){

// FORWARD SUBSTITUTION

        // Compute first nn elements
        for(unsigned int j = 0; j < nn; j++){
            for(unsigned int i = 0; i < j; i++){
                mu[0][j] = mu[0][j] - Beta[0][i][j]*mu[0][i];
            }
            mu[0][j] = Beta[0][j][j]*mu[0][j];
        }

        // Compute all the other elements except for the last nn
        for(unsigned int l = 1; l < NN-1; l++){
            for(unsigned int j = 0; j < nn; j++){
                for(unsigned int i = 0; i < nn; i++){
                    mu[l][j] = mu[l][j] - Alpha[l-1][i][j]*mu[l-1][i];
                }
                for(unsigned int i = 0; i < j; i++){
                    mu[l][j] = mu[l][j] - Beta[l][i][j]*mu[l][i];
                }
                mu[l][j] = Beta[l][j][j]*mu[l][j];
            }
        }

        // Compute the last nn elements
        for(unsigned int j = 0; j < nn; j++){
            for(unsigned int i = 0; i < nn; i++){
                mu[NN-1][j] = mu[NN-1][j] - Alpha[NN-2][i][j]*mu[NN-2][i];
            }
            for(unsigned int i = 0; i < j; i++){
                mu[NN-1][j] = mu[NN-1][j] - Beta[NN-1][i][j]*mu[NN-1][i];
            }
            mu[NN-1][j] = Beta[NN-1][j][j]*mu[NN-1][j];
        }

        // BACKWARD SUBSTITUTION

        // Compute the last nn elements
        for(unsigned int j = nn-1; j != -1; j--){
            for(unsigned int i = nn-1; i >= j+1; i--){
                mu[NN-1][j] = mu[NN-1][j] - Beta[NN-1][j][i]*mu[NN-1][i];
            }
            mu[NN-1][j] = Beta[NN-1][j][j]*mu[NN-1][j];
        }

        // Compute all the other elements except for the first nn
        for(unsigned int l = NN-2; l >=1; l--){
            for(unsigned int j = nn-1; j != -1; j--){
                for(unsigned int i = nn-1; i != -1; i--){
                    mu[l][j] = mu[l][j] - Alpha[l][j][i]*mu[l+1][i];
                }
                for(unsigned int i = nn-1; i >= j+1; i--){
                    mu[l][j] = mu[l][j] - Beta[l][j][i]*mu[l][i];
                }
                mu[l][j] = Beta[l][j][j]*mu[l][j];
            }
        }

        // Compute the first nn elements
        for(unsigned int j = nn-1; j != -1; j--){
            for(unsigned int i = nn-1; i != -1; i--){
                mu[0][j] = mu[0][j] - Alpha[0][j][i]*mu[1][i];
            }
            for(unsigned int i = nn-1; i >= j+1; i--){
                mu[0][j] = mu[0][j] - Beta[0][j][i]*mu[0][i];
            }
            mu[0][j] = Beta[0][j][j]*mu[0][j];
        }

}

