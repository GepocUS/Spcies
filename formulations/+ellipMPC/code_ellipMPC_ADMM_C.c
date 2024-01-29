/**
 * Sparse ADMM solver for the MPC formulation subject to terminal ellipsoidal constraint
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

void ellipMPC_ADMM(double *x0_in, double *xr_in, double *ur_in, double *u_opt, int *k_in, int *e_flag, sol_$INSERT_NAME$ *sol){

    #if MEASURE_TIME == 1

    #if WIN32
    static LARGE_INTEGER start, post_update, post_solve, post_polish;
    #else // If Linux
    struct timespec start, post_update, post_solve, post_polish;
    #endif

    read_time(&start);

    #endif

    // Initialize ADMM variables
    int done = 0; // Flag used to determine when the algorithm should exit
    int k = 0; // Number of iterations
    double x0[nn_]; // Current system state
    double xr[nn_]; // State reference
    double ur[mm_]; // Control input reference
    double v[NN_-1][nm_] = {{0.0}}; // Decision variables v
    double v_0[mm_] = {0.0};
    double v_N[nn_] = {0.0};
    double lambda[NN_-1][nm_] = {{0.0}}; // Dual variables lambda
    double lambda_0[mm_] = {0.0};
    double lambda_N[nn_] = {0.0};
    double z[NN_-1][nm_] = {{0.0}}; // Decision variables z
    double z_0[mm_] = {0.0};
    double z_N[nn_] = {0.0};
    double v1[NN_-1][nm_] = {{0.0}}; // Value of the decision variables z at the last iteration
    double v1_0[mm_] = {0.0};
    double v1_N[nn_] = {0.0};
    double aux_N[nn_] = {0.0}; // Auxiliary array used for multiple purposes
    double mu[NN_][nn_] = {{0.0}}; // Used to solve the system of equations
    unsigned int res_flag = 0; // Flag used to determine if the exit condition is satisfied
    double res_fixed_point; // Variable used to determine if a fixed point has been reached
    double res_primal_feas; // Variable used to determine if primal feasibility is satisfied
    double b[nn_] = {0.0}; // First nn_ components of vector b (the rest are known to be zero)
    double vPv; // Variable used to compute the P-projection onto the ellipsoid
    double q[nm_] = {0.0};
    double qT[nn_] = {0.0};
    $INSERT_VARIABLES$
    
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

    // Update first nn_ elements of beq
    for(unsigned int j = 0; j < nn_; j++){
        b[j] = 0.0;
        for(unsigned int i = 0; i < nn_; i++){
            b[j] = b[j] - AB[j][i]*x0[i];
        }
    }

    // Update the reference
    for(unsigned int j = 0; j < nn_; j++){
        q[j] = Q[j]*xr[j];
        qT[j] = 0.0;
        for(unsigned int i = 0; i < nn_; i++){
            qT[j] = qT[j] + T[j][i]*xr[i];
        }
    }
    for(unsigned int j = 0; j < mm_; j++){
        q[j+nn_] = R[j]*ur[j];
    }

    // Measure time
    #if MEASURE_TIME == 1
    read_time(&post_update);
    get_elapsed_time(&sol->update_time, &post_update, &start);
    #endif

    // Algorithm
    while(done == 0){

        k += 1; // Increment iteration counter

        // Step 0: Save the value of v into variable v1
        memcpy(v1_0, v_0, sizeof(double)*mm_);
        memcpy(v1, v, sizeof(double)*(NN_-1)*nm_);
        memcpy(v1_N, v_N, sizeof(double)*nn_);

        // Step 1: Minimize w.r.t. z_hat

        // Compute vector q_hat = q + lambda - rho*v
        // I store vector q_hat in z because I save memory and computation
        
        // Compute the first mm_ elements
        for(unsigned int j = 0; j < mm_; j++){
            #ifdef SCALAR_RHO
            z_0[j] = q[j+nn_] + lambda_0[j] - rho*v_0[j];
            #else
            z_0[j] = q[j+nn_] + lambda_0[j] - rho_0[j]*v_0[j];
            #endif
        }

        // Compute all the other elements except for the last nn_
        for(unsigned int l = 0; l < NN_-1; l++){
            for(unsigned int j = 0; j < nm_; j++){
                #ifdef SCALAR_RHO
                z[l][j] = q[j] + lambda[l][j] - rho*v[l][j];
                #else
                z[l][j] = q[j] + lambda[l][j] - rho[l][j]*v[l][j];
                #endif
            }
        }

        // Compute the last nn_ elements
        for(unsigned int j = 0; j < nn_; j++){
            z_N[j] = qT[j];
            for(unsigned int i = 0; i < nn_; i++){
                #ifdef SCALAR_RHO
                z_N[j] = z_N[j] + P_half[j][i]*lambda_N[i] - P[j][i]*rho*v_N[i];
                #else
                z_N[j] = z_N[j] + P_half[j][i]*lambda_N[i] - P[j][i]*rho_N[i]*v_N[i];
                #endif
            }
        }

        // Compute r.h.s of the Wc system of equations, i.e., -G'*H_hat^(-1)*q_hat - b
        // I store it in mu to save a bit of memory

        // Compute the first nn_ elements
        for(unsigned int j = 0; j < nn_; j++){
            mu[0][j] = Hi[0][j]*z[0][j] - b[j];
            for(unsigned int i = 0; i < mm_; i++){
                mu[0][j] = mu[0][j] - AB[j][i+nn_]*Hi_0[i]*z_0[i];
            }
        }

        // Compute all the other elements except for the last nn_
        for(unsigned int l = 1; l < NN_-1; l++){
            for(unsigned int j = 0; j < nn_; j++){
                mu[l][j] = Hi[l][j]*z[l][j];
                for(unsigned int i = 0; i < nm_; i++){
                    mu[l][j] = mu[l][j] - AB[j][i]*Hi[l-1][i]*z[l-1][i];
                }
            }
        }

        // Compute the last nn_ elements
        for(unsigned int j = 0; j < nn_; j++){
            mu[NN_-1][j] = 0.0;
            for(unsigned int i = 0; i < nn_; i++){
                mu[NN_-1][j] = mu[NN_-1][j] + Hi_N[j][i]*z_N[i];
            }
            for(unsigned int i = 0; i < nm_; i++){
                mu[NN_-1][j] = mu[NN_-1][j] - AB[j][i]*Hi[NN_-2][i]*z[NN_-2][i];
            }
        }
        
        // Compute mu, the solution of the system of equations W*mu = -G'*H^(-1)*q_hat - beq

        // FORWARD SUBSTITUTION

        // Compute first nn_ elements
        for(unsigned int j = 0; j < nn_; j++){
            for(unsigned int i = 0; i < j; i++){
                mu[0][j] = mu[0][j] - Beta[0][i][j]*mu[0][i];
            }
            mu[0][j] = Beta[0][j][j]*mu[0][j];
        }

        // Compute all the other elements except for the last nn_
        for(unsigned int l = 1; l < NN_-1; l++){
            for(unsigned int j = 0; j < nn_; j++){
                for(unsigned int i = 0; i < nn_; i++){
                    mu[l][j] = mu[l][j] - Alpha[l-1][i][j]*mu[l-1][i];
                }
                for(unsigned int i = 0; i < j; i++){
                    mu[l][j] = mu[l][j] - Beta[l][i][j]*mu[l][i];
                }
                mu[l][j] = Beta[l][j][j]*mu[l][j];
            }
        }

        // Compute the last nn_ elements
        for(unsigned int j = 0; j < nn_; j++){
            for(unsigned int i = 0; i < nn_; i++){
                mu[NN_-1][j] = mu[NN_-1][j] - Alpha[NN_-2][i][j]*mu[NN_-2][i];
            }
            for(unsigned int i = 0; i < j; i++){
                mu[NN_-1][j] = mu[NN_-1][j] - Beta[NN_-1][i][j]*mu[NN_-1][i];
            }
            mu[NN_-1][j] = Beta[NN_-1][j][j]*mu[NN_-1][j];
        }

        // BACKWARD SUBSTITUTION

        // Compute the last nn_ elements
        for(unsigned int j = nn_-1; j != -1; j--){
            for(unsigned int i = nn_-1; i >= j+1; i--){
                mu[NN_-1][j] = mu[NN_-1][j] - Beta[NN_-1][j][i]*mu[NN_-1][i];
            }
            mu[NN_-1][j] = Beta[NN_-1][j][j]*mu[NN_-1][j];
        }

        // Compute all the other elements except for the first nn_
        for(unsigned int l = NN_-2; l >=1; l--){
            for(unsigned int j = nn_-1; j != -1; j--){
                for(unsigned int i = nn_-1; i != -1; i--){
                    mu[l][j] = mu[l][j] - Alpha[l][j][i]*mu[l+1][i];
                }
                for(unsigned int i = nn_-1; i >= j+1; i--){
                    mu[l][j] = mu[l][j] - Beta[l][j][i]*mu[l][i];
                }
                mu[l][j] = Beta[l][j][j]*mu[l][j];
            }
        }

        // Compute the first nn_ elements
        for(unsigned int j = nn_-1; j != -1; j--){
            for(unsigned int i = nn_-1; i != -1; i--){
                mu[0][j] = mu[0][j] - Alpha[0][j][i]*mu[1][i];
            }
            for(unsigned int i = nn_-1; i >= j+1; i--){
                mu[0][j] = mu[0][j] - Beta[0][j][i]*mu[0][i];
            }
            mu[0][j] = Beta[0][j][j]*mu[0][j];
        }

        // Compute z (note that, from before, we have that at this point z = q_hat)

        // Compute the first mm_ elements
        for(unsigned int j = 0; j < mm_; j++){
            for(unsigned int i = 0; i < nn_; i++){
                z_0[j] = z_0[j] + AB[i][j+nn_]*mu[0][i];
            }
            z_0[j] = -Hi_0[j]*z_0[j];
        }

        // Compute all the other elements except for the last nn_
        for(unsigned int l = 0; l < NN_-1; l++){
            for(unsigned int j = 0; j < nn_; j++){
                z[l][j] = z[l][j] - mu[l][j];
            }
            for(unsigned int j = 0; j < nm_; j++){
                for(unsigned int i = 0; i < nn_; i++){
                    z[l][j] = z[l][j] + AB[i][j]*mu[l+1][i];
                }
                z[l][j] = -Hi[l][j]*z[l][j];
            }
        }

        // Compute the last nn_ elements
        for(unsigned int j = 0; j < nn_; j++){
            aux_N[j] = z_N[j] - mu[NN_-1][j];
        }
        for(unsigned int j = 0; j < nn_; j++){
            z_N[j] = 0.0;
            for(unsigned int i = 0; i < nn_; i++){
                z_N[j] = z_N[j] - Hi_N[j][i]*aux_N[i];
            }
        }

        // Step 2: Minimize w.r.t. v

        // Compute the first mm_ variables
        for(unsigned int j = 0; j < mm_; j++){
            #ifdef SCALAR_RHO
            v_0[j] = z_0[j] + rho_i*lambda_0[j];
            #else
            v_0[j] = z_0[j] + rho_i_0[j]*lambda_0[j];
            #endif
            v_0[j] = (v_0[j] > LBu0[j]) ? v_0[j] : LBu0[j]; // maximum between v and the lower bound
            v_0[j] = (v_0[j] > UBu0[j]) ? UBu0[j] : v_0[j]; // minimum between v and the upper bound
        }

        // Compute all the other elements except the last nn_
        for(unsigned int l = 0; l < NN_-1; l++){
            for(unsigned int j = 0; j < nm_; j++){
                #ifdef SCALAR_RHO
                v[l][j] = z[l][j] + rho_i*lambda[l][j];
                #else
                v[l][j] = z[l][j] + rho_i[l][j]*lambda[l][j];
                #endif
                v[l][j] = (v[l][j] > LBz[l][j]) ? v[l][j] : LBz[l][j]; // maximum between v and the lower bound
                v[l][j] = (v[l][j] > UBz[l][j]) ? UBz[l][j] : v[l][j]; // minimum between v and the upper bound
            }
        }

        // Compute the last nn_ elements

        // Compute the vector to be projected
        for(unsigned int j = 0; j < nn_; j++){
            v_N[j] = z_N[j];
            for(unsigned int i = 0; i < nn_; i++){
                #ifdef SCALAR_RHO
                v_N[j] = v_N[j] + Pinv_half[j][i]*rho_i*lambda_N[i];
                #else
                v_N[j] = v_N[j] + Pinv_half[j][i]*rho_i_N[i]*lambda_N[i];
                #endif
            }
        }

        // Compute (v_N - c)'*P*(v_N - c)
        for(unsigned int j = 0; j < nn_; j++){
            aux_N[j] = 0.0;
            for(unsigned int i = 0; i < nn_; i++){
                aux_N[j] = aux_N[j] + P[j][i]*( v_N[i] - c[i] );
            }
        }
        vPv = 0.0;
        for(unsigned int j = 0; j < nn_; j++){
            vPv = vPv + ( v_N[j] - c[j] )*aux_N[j];
        }

        // If v_N does not belong to the ellipsoid I need to perform the projection step
        if( vPv > r*r){
            vPv = r/sqrt(vPv);
            for(unsigned int j = 0; j < nn_; j++){
                v_N[j] = vPv*( v_N[j] - c[j] ) + c[j];
            }
        }

        // Step 3: Update lambda

        // Compute the first mm_ elements
        for(unsigned int j = 0; j < mm_; j++){
            #ifdef SCALAR_RHO
            lambda_0[j] = lambda_0[j] + rho*( z_0[j] - v_0[j] );
            #else
            lambda_0[j] = lambda_0[j] + rho_0[j]*( z_0[j] - v_0[j] );
            #endif
        }

        // Compute all the other elements except for the last nn_
        for(unsigned int l = 0; l < NN_-1; l++){
            for(unsigned int j = 0; j < nm_; j++){
                #ifdef SCALAR_RHO
                lambda[l][j] = lambda[l][j] + rho*( z[l][j] - v[l][j] );
                #else
                lambda[l][j] = lambda[l][j] + rho[l][j]*( z[l][j] - v[l][j] );
                #endif
            }
        }

        // Compute the last nn_ elements
        for(unsigned int j = 0; j < nn_; j++){
            #ifdef SCALAR_RHO
            aux_N[j] = rho*(z_N[j] - v_N[j]);
            #else
            aux_N[j] = rho_N[j]*(z_N[j] - v_N[j]);
            #endif
        }
        for(unsigned int j = 0; j < nn_; j++){
            for(unsigned int i = 0; i < nn_; i++){
                lambda_N[j] = lambda_N[j] + P_half[j][i]*aux_N[i];
            }
        }

        // Step 4: Compute the residual

        res_flag = 0; // Reset the residual flag

        // Compute the first mm_ elements
        for(unsigned int j = 0; j < mm_; j++){
            res_fixed_point = v1_0[j] - v_0[j];
            res_primal_feas = z_0[j] - v_0[j];
            // Obtain absolute values
            res_fixed_point = ( res_fixed_point > 0.0 ) ? res_fixed_point : -res_fixed_point;
            res_primal_feas = ( res_primal_feas > 0.0 ) ? res_primal_feas : -res_primal_feas;
            if( res_fixed_point > tol || res_primal_feas > tol){
                res_flag = 1;
                break;
            }
        }

        // Compute the last NN_ elements
        if(res_flag == 0){
            for(unsigned int j = 0; j < nn_; j++){
                res_fixed_point = v1_N[j] - v_N[j];
                res_primal_feas = z_N[j] - v_N[j];
                // Obtain absolute values
                res_fixed_point = ( res_fixed_point > 0.0 ) ? res_fixed_point : -res_fixed_point;
                res_primal_feas = ( res_primal_feas > 0.0 ) ? res_primal_feas : -res_primal_feas;
                if( res_fixed_point > tol || res_primal_feas > tol){
                    res_flag = 1;
                    break;
                }
            }
        }

        // Compute all the other elements
        if(res_flag == 0){
            for(unsigned int l = 0; l < NN_-1; l++){
                for(unsigned int j = 0; j < nm_; j++){
                    res_fixed_point = v1[l][j] - v[l][j];
                    res_primal_feas = z[l][j] - v[l][j];
                    // Obtain absolute values
                    res_fixed_point = ( res_fixed_point > 0.0 ) ? res_fixed_point : -res_fixed_point;
                    res_primal_feas = ( res_primal_feas > 0.0 ) ? res_primal_feas : -res_primal_feas;
                    if( res_fixed_point > tol || res_primal_feas > tol){
                        res_flag = 1;
                        break;
                    }
                }
                if(res_flag == 1){
                    break;
                }
            }
        }

        // Step 5: Exit condition

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
        u_opt[j] = v_0[j]*scaling_i_u[j] + OpPoint_u[j];
    }
    #endif
    #if in_engineering == 0
    for(unsigned int j = 0; j < mm_; j++){
        u_opt[j] = v_0[j];
    }
    #endif

    // Return number of iterations
    *k_in = k; // Number of iterations

    // Save solution into structure
    #ifdef DEBUG

    // First mm_ variables
    int count = -1;
    for(unsigned int j = 0; j < mm_; j++){
        count++;
        sol->z[count] = z_0[j];
        sol->v[count] = v_0[j];
        sol->lambda[count] = lambda_0[j];
    }

    // All other elements except the last nn_
    for(unsigned int l = 0; l < NN_-1; l++){
        for(unsigned int j = 0; j < nm_; j++){
            count++;
            sol->z[count] = z[l][j];
            sol->v[count] = v[l][j];
            sol->lambda[count] = lambda[l][j];
        }
    }

    // Last nn_ elements
    for(unsigned int j = 0; j < nn_; j++){
        count++;
        sol->z[count] = z_N[j];
        sol->v[count] = v_N[j];
        sol->lambda[count] = lambda_N[j];
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

