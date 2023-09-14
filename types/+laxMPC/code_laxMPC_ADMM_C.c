/**
 * Sparse ADMM solver for the lax MPC formulation
 *
 * ARGUMENTS:
 * The current system state is given in "x0_in". Pointer to array of size nn_.
 * The state reference is given in "xr_in". Pointer to array of size nn_.
 * The input reference is given in "ur_in". Pointer to array of size mm.
 * The optimal control action is returned in "u_opt". Pointer to array of size mm.
 * The number of iterations is returned in "k_in". Pointer to int.
 * The exit flag is returned in "e_flag". Pointer to int.
 *       1: Algorithm converged successfully.
 *      -1: Algorithm did not converge within the maximum number of iterations. Returns current iterate.
 * The optimal decision variables and dual variables are returned in the solution structure sol.
 *
 */

#include <stdio.h>

#if MEASURE_TIME == 1

#if WIN32
#include <Windows.h>
#else // If Linux
#include <time.h>
#endif

#endif
    
#if TIME_VARYING == 1
void laxMPC_ADMM(double *x0_in, double *xr_in, double *ur_in, double *A_in, double *B_in, double *Q_in, double *R_in, double *LB_in, double *UB_in, double *u_opt, int *k_in, int *e_flag, solution *sol){
#else
void laxMPC_ADMM(double *x0_in, double *xr_in, double *ur_in, double *u_opt, int *k_in, int *e_flag, solution *sol){
#endif

    #if MEASURE_TIME == 1

    #if WIN32
    static LARGE_INTEGER frequency, start, post_update, post_solve, post_polish;
    __int64 t_update_time, t_solve_time, t_polish_time, t_run_time; // Time in nano-seconds

    if (frequency.QuadPart == 0){
    QueryPerformanceFrequency(&frequency);}

    QueryPerformanceCounter(&start); // Get time at the start

    #else // If Linux
    // Initialize time variables
    struct timespec start, post_update, post_solve, post_polish;
    clock_gettime(CLOCK_MONOTONIC_RAW, &start);
    #endif

    #endif

    // Initialize ADMM variables
    int done = 0; // Flag used to determine when the algorithm should exit
    int k = 0; // Number of iterations
    double x0[nn_]; // Current system state
    double xr[nn_]; // State reference
    double ur[mm_]; // Control input reference
    #if TIME_VARYING == 1
        double AB[nn_][nm_];
        double Q[nn_]; // Weight matrix for the states
        double R[mm_]; // Weight matrix for the inputs
        double Hi[NN_-1][nm_]; // Inverse of the Hessian H
        double Hi_0[mm_];
        double Hi_N[nn_][nn_];
        double R_rho_i[mm_] = {0.0}; // 1./(R+rho*I) // Needed for calculating Alpha and Beta online
        double Q_rho_i[nn_] = {0.0}; // 1./(Q+rho*I
        double AQiAt[nn_][nn_] = {{0.0}}; // A*inv(Q+rho*I)*A'
        double BRiBt[nn_][nn_] = {{0.0}}; // B*inv(R+rho*I)*B'
        double Alpha[NN_-1][nn_][nn_] = {{{0.0}}}; // Variables used for solving the equality constrained QP
        double Beta[NN_][nn_][nn_] = {{{0.0}}}; // Variables used for solving the equality constrained QP. Note: Diagonal of Beta's are inverted
        double LB[nm_]; // Lower bound for box constraints 
        double UB[nm_]; // Upper bound for box constraints
    #endif
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
    double q[nm_] = {0.0};
    double qT[nn_] = {0.0};
    
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
    #if TIME_VARYING == 1
    for(unsigned int i=0; i < nn_; i++){ 
        LB[i] = scaling_x[i]*( LB_in[i] - OpPoint_x[i] );
        UB[i] = scaling_x[i]*( UB_in[i] - OpPoint_x[i] );
    }
    for(unsigned int i=0; i < mm_; i++){ 
        LB[i+nn_] = scaling_u[i]*( LB_in[i+nn_] - OpPoint_u[i] );
        UB[i+nn_] = scaling_u[i]*( UB_in[i+nn_] - OpPoint_u[i] );
    }
    #endif
    #else
    for(unsigned int i = 0; i < nn_; i++){
        x0[i] = x0_in[i];
        xr[i] = xr_in[i];
    }
    for(unsigned int i = 0; i < mm_; i++){
        ur[i] = ur_in[i];
    }
    #if TIME_VARYING == 1
    for(unsigned int i=0; i < nm_; i++){ 
        LB[i] = LB_in[i];
        UB[i] = UB_in[i];
    }
    #endif
    #endif

    #if TIME_VARYING == 1
    for(unsigned int i = 0; i < nn_; i++){
        Q[i] = Q_in[i];
        Q_rho_i[i] = 1/(Q[i]+rho);
        for(unsigned int j = 0; j < nn_; j++){
            AB[i][j] = A_in[i+j*nn_]; // Add A to AB
            Hi_N[i][j] = T_rho_i[i][j];
        }
        for(unsigned int j=0; j < mm_; j++){
            AB[i][nn_+j] = B_in[i+j*nn_]; // Add B to AB
        }
    }
    for(unsigned int j=0; j < mm_; j++){ 
        R[j] = R_in[j];
        R_rho_i[j] = 1/(R[j]+rho);
        Hi_0[j] = R_rho_i[j];
    }

    // Constructing Hi
    for(unsigned int i=0 ; i<NN_-1 ; i++){
        for(unsigned int j=0 ; j<nm_ ; j++){
        // FIX: This can be improved by using two loops
            if(j<nn_){
                Hi[i][j] = Q_rho_i[j];
            }
            else{
                Hi[i][j] = R_rho_i[j-nn_];
            }
        }
    }
    #endif
    
    // Computation of Alpha and Beta
    #if TIME_VARYING == 1

    // FIX: We shouldn't be using A_in and B_in any more. Instead, we should use AB[][]
    for(unsigned int i = 0 ; i<nn_ ; i++){
        for(unsigned int j=0 ; j<nn_ ; j++){
            for(unsigned int k=0 ; k<nn_ ; k++){
                AQiAt[i][j] += A_in[i+k*nn_]*Q_rho_i[k]*A_in[j+k*nn_];
            }
            for (unsigned int m = 0 ; m < mm_ ; m++){
                BRiBt[i][j] += B_in[i+m*nn_]*R_rho_i[m]*B_in[j+m*nn_];
            }
        }
    }

    //Beta{0}
    for(unsigned int i = 0 ; i < nn_ ; i++){
        for(unsigned int j = i ; j < nn_ ; j++){
            
            Beta[0][i][j] = BRiBt[i][j];

            if(i>0){
                for(unsigned int l = 0 ; l <= i-1 ; l++){
                    Beta[0][i][j] -= Beta[0][l][i]*Beta[0][l][j];
                }               
            }
            if (i==j){
                Beta[0][i][i] += Q_rho_i[i];
                Beta[0][i][i] = 1/sqrt(Beta[0][i][i]);
            }
            else{ 
                Beta[0][i][j] = Beta[0][i][j]*Beta[0][i][i];
            }
        }
    }

    // ALpha{0}
    for(unsigned int i=0 ; i < nn_ ; i++){
        for(unsigned int j=0 ; j < nn_ ; j++){

            Alpha[0][i][j] = -Q_rho_i[i]*AB[j][i];
            
            if(i>0){
                for(unsigned int l=0 ; l <= i-1 ; l++){
                    Alpha[0][i][j] -= Beta[0][l][i] * Alpha[0][l][j];
                }
            }

            Alpha[0][i][j] = Alpha[0][i][j]*Beta[0][i][i];

        }
    }


    // Beta{1} to Beta{N-2}
    for(unsigned int h = 1; h < NN_-1 ; h++){
        for(unsigned int i = 0 ; i < nn_ ; i++){
            for(unsigned int j = i ; j < nn_ ; j++){

                Beta[h][i][j] = AQiAt[i][j] + BRiBt[i][j];

                for(unsigned int k = 0 ; k < nn_ ; k++){
                    Beta[h][i][j] -= Alpha[h-1][k][i]*Alpha[h-1][k][j];
                }
                        
                if(i>0){
                    for(unsigned int l = 0 ; l <= i-1 ; l++){
                        Beta[h][i][j] -= Beta[h][l][i]*Beta[h][l][j];
                    }
                }
                    
                if(i==j){
                    Beta[h][i][i] += Q_rho_i[i];   
                    Beta[h][i][i] = 1/sqrt(Beta[h][i][i]);
                }
                else {
                    Beta[h][i][j] = Beta[h][i][j]*Beta[h][i][i];
                }
            
            }
        }
        
        //Alpha[h][][]
        for(unsigned int i=0 ; i < nn_ ; i++){
            for(unsigned int j=0 ; j < nn_ ; j++){

                Alpha[h][i][j] = -Q_rho_i[i]*AB[j][i];
                
                if(i>0){
                    for(unsigned int l=0 ; l <= i-1 ; l++){
                        Alpha[h][i][j] -= Beta[h][l][i] * Alpha[h][l][j];
                    }
                }

                Alpha[h][i][j] = Alpha[h][i][j]*Beta[h][i][i];

            }
        }

    }

    // Beta{N-1}

    for(unsigned int i = 0 ; i < nn_ ; i++){
        for(unsigned int j = i ; j < nn_ ; j++){

            Beta[NN_-1][i][j] = AQiAt[i][j] + BRiBt[i][j];

            for(unsigned int k = 0 ; k < nn_ ; k++){
                Beta[NN_-1][i][j] -= Alpha[NN_-2][k][i]*Alpha[NN_-2][k][j];
            }
                        
            if(i>0){
                for(unsigned int l = 0 ; l <= i-1 ; l++){
                    Beta[NN_-1][i][j] -= Beta[NN_-1][l][i]*Beta[NN_-1][l][j];
                }
            }

            Beta[NN_-1][i][j] += T_rho_i[i][j]; // When calculating Beta{N}, as T_rho_i is dense, we add it in every component instead of only in the diagonal as in Beta{0} to Beta{N-2}

            if(i==j){    
                Beta[NN_-1][i][i] = 1/sqrt(Beta[NN_-1][i][i]);
            }

            else{
                Beta[NN_-1][i][j] = Beta[NN_-1][i][j]*Beta[NN_-1][i][i];
            }

        }
    }
    // End of computation of Alpha and Beta

    // Multiply Q and R by -1, since this value is used the rest of the algorithm
    for(unsigned int i=0 ; i<nn_ ; i++){
        Q[i] = -Q[i];
    }
    for(unsigned int i=0 ; i<mm_ ; i++){
        R[i] = -R[i];
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

    #if WIN32
    QueryPerformanceCounter(&post_update); // Get time after the update    
    t_update_time = 1000000000ULL * (post_update.QuadPart - start.QuadPart) / frequency.QuadPart;
    sol->update_time = t_update_time/(double)1e+9;
    #else // If Linux
    clock_gettime(CLOCK_MONOTONIC_RAW, &post_update);
    sol->update_time = (double) ( (post_update.tv_sec - start.tv_sec) * 1000.0 ) + (double) ( (post_update.tv_nsec - start.tv_nsec) / 1000000.0 );
    #endif

    #endif

    // Algorithm
    while(done == 0){

        k += 1; // Increment iteration counter

        // Step 0: Save the value of v into variable v1
        memcpy(v1_0, v_0, sizeof(double)*mm_);
        memcpy(v1, v, sizeof(double)*(NN_-1)*nm_);
        memcpy(v1_N, v_N, sizeof(double)*nn_);

        // Step 1: Minimize w.r.t. z

        // Compute vector q_hat = lambda - rho*v
        // I store vector q_hat in z because I save memory and computation
        
        // Compute the first mm elements
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
            #ifdef SCALAR_RHO
            z_N[j] = qT[j] + lambda_N[j] - rho*v_N[j];
            #else
            z_N[j] = qT[j] + lambda_N[j] - rho_N[j]*v_N[j];
            #endif
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

        // Compute the first mm elements
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

        // Compute the first mm variables
        for(unsigned int j = 0; j < mm_; j++){
            #ifdef SCALAR_RHO
            v_0[j] = z_0[j] + rho_i*lambda_0[j];
            #else
            v_0[j] = z_0[j] + rho_i_0[j]*lambda_0[j];
            #endif
            #ifdef VAR_BOUNDS
            v_0[j] = (v_0[j] > LB0[j]) ? v_0[j] : LB0[j]; // maximum between v and the lower bound
            v_0[j] = (v_0[j] > UB0[j]) ? UB0[j] : v_0[j]; // minimum between v and the upper bound
            #else
            v_0[j] = (v_0[j] > LB[j+nn_]) ? v_0[j] : LB[j+nn_]; // maximum between v and the lower bound
            v_0[j] = (v_0[j] > UB[j+nn_]) ? UB[j+nn_] : v_0[j]; // minimum between v and the upper bound
            #endif
        }

        // Compute all the other elements except the last nn_
        for(unsigned int l = 0; l < NN_-1; l++){
            for(unsigned int j = 0; j < nm_; j++){
                #ifdef SCALAR_RHO
                v[l][j] = z[l][j] + rho_i*lambda[l][j];
                #else
                v[l][j] = z[l][j] + rho_i[l][j]*lambda[l][j];
                #endif
                #ifdef VAR_BOUNDS
                v[l][j] = (v[l][j] > LB[l][j]) ? v[l][j] : LB[l][j]; // maximum between v and the lower bound
                v[l][j] = (v[l][j] > UB[l][j]) ? UB[l][j] : v[l][j]; // minimum between v and the upper bound
                #else
                v[l][j] = (v[l][j] > LB[j]) ? v[l][j] : LB[j]; // maximum between v and the lower bound
                v[l][j] = (v[l][j] > UB[j]) ? UB[j] : v[l][j]; // minimum between v and the upper bound
                #endif
            }
        }

        // Compute the last nn_ elements
        for(unsigned int j = 0; j < nn_; j++){
            #ifdef SCALAR_RHO
            v_N[j] = z_N[j] + rho_i*lambda_N[j];
            #else
            v_N[j] = z_N[j] + rho_i_N[j]*lambda_N[j];
            #endif
            #ifdef VAR_BOUNDS
            v_N[j] = (v_N[j] > LBN[j]) ? v_N[j] : LBN[j]; // maximum between v and the lower bound
            v_N[j] = (v_N[j] > UBN[j]) ? UBN[j] : v_N[j]; // minimum between v and the upper bound
            #else
            v_N[j] = (v_N[j] > LB[j]) ? v_N[j] : LB[j]; // maximum between v and the lower bound
            v_N[j] = (v_N[j] > UB[j]) ? UB[j] : v_N[j]; // minimum between v and the upper bound
            #endif
        }

        // Step 3: Update lambda

        // Compute the first mm elements
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
            lambda_N[j] = lambda_N[j] + rho*( z_N[j] - v_N[j] );
            #else
            lambda_N[j] = lambda_N[j] + rho_N[j]*( z_N[j] - v_N[j] );
            #endif
        }

        // Step 4: Compute the residual

        res_flag = 0; // Reset the residual flag

        // Compute the first mm elements
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
    
    #if WIN32
    QueryPerformanceCounter(&post_solve); // Get time after solving
    t_solve_time = 1000000000ULL * (post_solve.QuadPart - post_update.QuadPart) / frequency.QuadPart;
    sol->solve_time = t_solve_time/(double)1e+9;
    #else // If Linux
    clock_gettime(CLOCK_MONOTONIC_RAW, &post_solve);
    sol->solve_time = (double) ( (post_solve.tv_sec - post_update.tv_sec) * 1000.0 ) + (double) ( (post_solve.tv_nsec - post_update.tv_nsec) / 1000000.0 );
    #endif
    
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

    // First mm variables
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
    
    #if WIN32
    QueryPerformanceCounter(&post_polish); // Get time after polishing
    t_run_time = 1000000000ULL * (post_polish.QuadPart - start.QuadPart) / frequency.QuadPart;
    t_polish_time = 1000000000ULL * (post_polish.QuadPart - post_solve.QuadPart) / frequency.QuadPart;
    sol->run_time = t_run_time/(double)1e+9;
    sol->polish_time = t_polish_time/(double)1e+9;
    #else // If Linux
    clock_gettime(CLOCK_MONOTONIC_RAW, &post_polish);
    sol->run_time = (double) ( (post_polish.tv_sec - start.tv_sec) * 1000.0 ) + (double) ( (post_polish.tv_nsec - start.tv_nsec) / 1000000.0 );
    sol->polish_time = (double) ( (post_polish.tv_sec - post_solve.tv_sec) * 1000.0 ) + (double) ( (post_polish.tv_nsec - post_solve.tv_nsec) / 1000000.0 );
    #endif
    
    #endif

}

