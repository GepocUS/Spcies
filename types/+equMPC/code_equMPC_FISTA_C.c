/**
 * Sparse FISTA solver for the equality MPC formulation
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

#ifdef MEASURE_TIME

#if WIN32
#include <Windows.h>
#else // If Linux
#include <time.h>
#endif

#endif

#if TIME_VARYING  == 1
void equMPC_FISTA(double *x0_in, double *xr_in, double *ur_in, double *A_in, double *B_in, double *Q_in, double *R_in, double *LB_in, double *UB_in, double *u_opt, int *k_in, int *e_flag, sol_equMPC_FISTA *sol){
#else
void equMPC_FISTA(double *x0_in, double *xr_in, double *ur_in, double *u_opt, int *k_in, int *e_flag, sol_equMPC_FISTA *sol){
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

    // Initialize FISTA variables
    int done = 0; // Flag used to determine when the algorithm should exit
    int k = 0; // Number of iterations
    double x0[nn_]; // Current system state
    double xr[nn_]; // State reference
    double ur[mm_]; // Control input reference
    #if TIME_VARYING == 1
    double AB[nn_][nm_];
    double Q[nn_];
    double R[mm_];
    double QRi[nm_];
    double R_i[mm_]; // 1./(diag(R)) Needed for calculation of Alpha's and Beta's online
    double Q_i[nn_]; // 1./(diag(Q)) Needed for calculation of Alpha's and Beta's online
    double AQiAt[nn_][nn_] = {{0.0}}; // A*inv(Q+rho*I)*A'
    double BRiBt[nn_][nn_] = {{0.0}}; // B*inv(R+rho*I)*B'
    double Alpha[NN_-1][nn_][nn_] = {{{0.0}}}; // Static because they need to go into functions which use their value
    double Beta[NN_][nn_][nn_] = {{{0.0}}};
    double inv_Beta[nn_][nn_] = {{0.0}}; // Inverse of only the current beta is stored
    double LB[nm_]; // Lower bound for box constraints 
    double UB[nm_]; // Upper bound for box constraints
    #endif
    double z[NN_-1][nm_] = {{0.0}}; // Primal decision variables
    double z_0[mm_] = {0.0};
    double y[NN_][nn_] = {{0.0}}; // Linearization point
    double lambda[NN_][nn_] = {{0.0}}; // Dual variables
    double lambda1[NN_][nn_] = {{0.0}}; // Dual variables in the previous iteration
    double d_lambda[NN_][nn_] = {{0.0}}; // Dual variables
    double t = 1.0; // Step of the FISTA algorithm
    double t1 = 1.0; // Step of the previous iteration
    double b[nn_] = {0.0}; // First nn_ components of vector b (the rest are known to be zero)
    double q[nm_] = {0.0}; // For storing the components of q related to Q and R
    double res = 0.0; // For storing the absolute value of each element of the residual vector
    unsigned int res_flag = 0; // Flag used to determine if the exit condition is satisfied

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
    for(unsigned int i = 0 ; i<nn_ ; i++){
        Q[i] = Q_in[i];
        Q_i[i] = 1/(Q[i]);
        for(unsigned int j = 0; j < nn_; j++){
            AB[i][j] = A_in[i+j*nn_]; // Add A to AB
        }
        for(unsigned int j=0; j < mm_; j++){
            AB[i][nn_+j] = B_in[i+j*nn_]; // Add B to AB
        }
    }
    for(unsigned int j=0; j < mm_; j++){ 
        R[j] = R_in[j];
        R_i[j] = 1/(R[j]);
    }

    // Constructing QRi
    // FIX: This can be improved by using two loops
    for(unsigned int i = 0 ; i<nn_+mm_ ; i++){
        if (i<nn_){
            QRi[i] = -Q_i[i];
        }
        else{
            QRi[i] = -R_i[i-nn_];
        }
    }
    #endif

    // Computation of Alpha and Beta
    #if TIME_VARYING == 1
    
    memset(Beta, 0, sizeof(Beta)); // These two lines solve problems because now Alpha and Beta are static, and otherwise they remember their last value, what leads to errors
    memset(Alpha, 0, sizeof(Alpha)); // Without this, that happens even though in every call they are declared as zeros.
    memset(AQiAt, 0, sizeof(AQiAt));
    memset(BRiBt, 0, sizeof(BRiBt));

    // FIX: We shouldn't be using A_in and B_in any more. Instead, we should use AB[][]
    for(unsigned int i = 0 ; i<nn_ ; i++){
        for(unsigned int j=0 ; j<nn_ ; j++){
            for(unsigned int k=0 ; k<nn_ ; k++){
                AQiAt[i][j] += A_in[i+k*nn_]*Q_i[k]*A_in[j+k*nn_];
            }
            for (unsigned int m = 0 ; m < mm_ ; m++){
                BRiBt[i][j] += B_in[i+m*nn_]*R_i[m]*B_in[j+m*nn_];
            }
        }
    }

    // Beta{0}
    for(unsigned int i = 0 ; i < nn_ ; i++){
        for(unsigned int j = 0 ; j < nn_ ; j++){

            Beta[0][i][j] = BRiBt[i][j];
                
            if(i>0){
                for(unsigned int l = 0 ; l <= i-1 ; l++){
                    Beta[0][i][j] -= Beta[0][l][i]*Beta[0][l][j];
                }               
            }
            if (i==j){
                Beta[0][i][j] += Q_i[i];
                Beta[0][i][j] = sqrt(Beta[0][i][j]);
            }
            else{ 
                Beta[0][i][j] = Beta[0][i][j]/Beta[0][i][i];
            }
        }
    }

    // Inverse of Beta{0}
    memset(inv_Beta, 0, sizeof(inv_Beta)); // Reset of inv_Beta when a new Beta is calculated
    for (int i=nn_-1 ; i>=0 ; i--){
        for (unsigned int j=0 ; j<nn_ ; j++){
            if(i==j){
                inv_Beta[i][i] = 1/Beta[0][i][i]; // Calculation of diagonal elements
            }
            else if (j>i){
                for(unsigned int k = i+1 ; k<=j ; k++){
                    inv_Beta[i][j] += Beta[0][i][k]*inv_Beta[k][j];
            }
                inv_Beta[i][j] = -1/Beta[0][i][i]*inv_Beta[i][j];
            }
        }
    }

    // Alpha{0}
    for (unsigned int i=0 ; i<nn_ ; i++){
        for (unsigned int j=0 ; j<nn_ ; j++){
            for (unsigned int k=0 ; k<=i ; k++){
                Alpha[0][i][j] -= inv_Beta[k][i] * A_in[j+k*nn_] * Q_i[k];
            }
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
                    for(unsigned int l = 0 ; l<=i-1 ; l++){
                        Beta[h][i][j] -= Beta[h][l][i]*Beta[h][l][j];
                    }
                }

                if(i==j){
                    Beta[h][i][j] += Q_i[i];
                    Beta[h][i][j] = sqrt(Beta[h][i][j]);
                }
                else{
                    Beta[h][i][j] = Beta[h][i][j]/Beta[h][i][i];
                }

            }

        }
        // Calculation of the inverse of the current Beta, needed for current Alpha
        memset(inv_Beta, 0, sizeof(inv_Beta)); // Reset of inv_Beta when a new Beta is calculated

        for (int i=nn_-1 ; i>=0 ; i--){
            for (unsigned int j=0 ; j<nn_ ; j++){
                if(i==j){
                    inv_Beta[i][i] = 1/Beta[h][i][i]; // Calculation of diagonal elements
                }
                else if (j>i){
                    for(unsigned int k = i+1 ; k<=j ; k++){
                        inv_Beta[i][j] += Beta[h][i][k]*inv_Beta[k][j];
                    }
                    inv_Beta[i][j] = -1/Beta[h][i][i]*inv_Beta[i][j];
                }
            }
        }
        // Calculation of current Alpha
        for (unsigned int i=0 ; i<nn_ ; i++){
            for (unsigned int j=0 ; j<nn_ ; j++){
                for (unsigned int k=0 ; k<=i ; k++){
                    Alpha[h][i][j] -= inv_Beta[k][i] * A_in[j+k*nn_] * Q_i[k];
                }
            }
        }
    }

    //Beta{N-1}
    for(unsigned int i = 0 ; i < nn_ ; i++){
        for(unsigned int j = i ; j < nn_ ; j++){

            Beta[NN_-1][i][j] = AQiAt[i][j] + BRiBt[i][j];
            for(unsigned int k=0 ; k<nn_ ; k++){
                Beta[NN_-1][i][j] -= Alpha[NN_-2][k][i]*Alpha[NN_-2][k][j];
            }

            if(i>0){
                for(unsigned int l=0 ; l<=i-1 ; l++){
                    Beta[NN_-1][i][j] -= Beta[NN_-1][l][i] * Beta[NN_-1][l][j];
                }
            }
            if(i==j){
                Beta[NN_-1][i][j] = sqrt(Beta[NN_-1][i][j]);
            }
            else{
                Beta[NN_-1][i][j] = Beta[NN_-1][i][j]/Beta[NN_-1][i][i];
            }
            
        }
    }

    for (unsigned int h=0 ; h<NN_ ; h++){

        for (unsigned int i=0 ; i<nn_ ; i++){
            Beta[h][i][i] = 1/Beta[h][i][i]; // We need to make the component-wise inversion of the diagonal elements of Beta's
        }
        
    }
    // End of computation of Alpha and Beta
    
    
    // Inverting Q and R, needed for the rest of the program
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

    // Initial steps 

    // Compute z_lambda_0
    compute_z_lambda_equMPC_FISTA(z_0, z, lambda, q, QRi, AB, LB, UB);

    // Compute the residual: We save it into d_lambda to save memory and future computations
    compute_residual_vector_equMPC_FISTA(d_lambda, z_0, z, b, xr, AB);

    // Compute delta_lambda_0
    solve_W_matrix_form(d_lambda, Alpha, Beta);

    // Update lambda_0
    for(unsigned int l = 0; l < NN_; l++){
        for(unsigned int j = 0; j < nn_; j++){
            lambda[l][j] = lambda[l][j] + d_lambda[l][j];
        }
    }

    // Update y_0
    for(unsigned int l = 0; l < NN_; l++){
        for(unsigned int j = 0; j < nn_; j++){
            y[l][j] = lambda[l][j];
        }
    }

    // Algorithm
    while(done == 0){

        k += 1; // Increment iteration counter

        // Step 0: Save the value of lambda into variable lambda1 and t into t1
        memcpy(lambda1, lambda, sizeof(double)*NN_*nn_);
        t1 = t;

        // Compute z_lambda_k
        compute_z_lambda_equMPC_FISTA(z_0, z, y, q, QRi, AB, LB, UB);

        // Compute the residual: We save it into d_lambda to save memory and future computations
        compute_residual_vector_equMPC_FISTA(d_lambda, z_0, z, b, xr, AB);

        // Check exit condition
        res_flag = 0; // Reset exit flag

        for(unsigned int l = 0; l < NN_; l++){
            for(unsigned int j = 0; j < nn_; j++){
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
            *e_flag = 1;
        }
        else if( k >= k_max ){
            done = 1;
            *e_flag = -1;
        }

        // The rest of the steps are unn_ecessary if done == 1

        if( done == 0 ){

            // Compute delta_lambda_k
            solve_W_matrix_form(d_lambda, Alpha, Beta);

            // Update lambda_k
            for(unsigned int l = 0; l < NN_; l++){
                for(unsigned int j = 0; j < nn_; j++){
                    lambda[l][j] = y[l][j] + d_lambda[l][j];
                }
            }

            // Update t_k
            t = 0.5*( 1 + sqrt( 1 + 4*t1*t1) );

            // Update y_k
            for(unsigned int l = 0; l < NN_; l++){
                for(unsigned int j = 0; j < nn_; j++){
                    y[l][j] = lambda[l][j] + (t1 - 1)*(lambda[l][j] - lambda1[l][j])/t;
                }
            }

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
        u_opt[j] = z_0[j]*scaling_i_u[j] + OpPoint_u[j];
    }
    #endif
    #if in_engineering == 0
    for(unsigned int j = 0; j < mm_; j++){
        u_opt[j] = z_0[j];
    }
    #endif

    // Return number of iterations
    *k_in = k; // Number of iterations

    // Save solution into structure
    #ifdef DEBUG

    // Return z_opt

    // First mm variables
    int count = -1;
    for(unsigned int j = 0; j < mm_; j++){
        count++;
        sol->z[count] = z_0[j];
    }

    // All other elements
    for(unsigned int l = 0; l < NN_-1; l++){
        for(unsigned int j = 0; j < nm_; j++){
            count++;
            sol->z[count] = z[l][j];
        }
    }

    // Return lambda_opt
    count = -1;
    for(unsigned int l = 0; l < NN_; l++){
        for(unsigned int j = 0; j < nn_; j++){
            count++;
            sol->lambda[count] = y[l][j];
        }
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

/* compute_z_lambda_equMPC_FISTA()
*
*/

void compute_z_lambda_equMPC_FISTA(double *z_0, double z[][nm_], double lambda[][nn_], double *q, const double *QRi, const double (*AB)[nm_], const double *LB, const double *UB){

    // Compute first mm elements
    for(unsigned int j = 0; j < mm_; j++){

        // Compute the q - G'*lambda part
        z_0[j] = q[j+nn_];
        for(unsigned int i = 0; i < nn_; i++){
            z_0[j] = z_0[j] - AB[i][j+nn_]*lambda[0][i];
        }

        // Compute the solution of the QP
        z_0[j] = z_0[j]*QRi[j+nn_]; // Multiply by the inverse of the Hessian
        #ifdef VAR_BOUNDS
        z_0[j] = (z_0[j] > LB0[j]) ? z_0[j] : LB0[j]; // maximum between v and the lower bound
        z_0[j] = (z_0[j] > UB0[j]) ? UB0[j] : z_0[j]; // minimum between v and the upper bound
        #else
        z_0[j] = (z_0[j] > LB[j+nn_]) ? z_0[j] : LB[j+nn_]; // maximum between v and the lower bound
        z_0[j] = (z_0[j] > UB[j+nn_]) ? UB[j+nn_] : z_0[j]; // minimum between v and the upper bound
        #endif
    }

    // Compute all the other elements except the last nn_
    for(unsigned int l = 0; l < NN_-1; l++){

        // Compute the q - G'*lambda part
        for(unsigned int j = 0; j < nm_; j++){
            z[l][j] = q[j];
            for(unsigned int i = 0; i < nn_; i++){
                z[l][j] = z[l][j] - AB[i][j]*lambda[l+1][i];
            }
        }
        for(unsigned int j = 0; j < nn_; j++){
            z[l][j] = z[l][j] + lambda[l][j];
        }

        // Compute the solution of the QP
        for(unsigned int j = 0; j < nm_; j++){
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

    // Compute the last nn_ elements
    // for(unsigned int j = 0; j < nn_; j++){

        // Compute the q - G'*lambda part
        // z_N[j] = qP[j] + lambda[NN_-1][j];

        // Compute the solution of the QP
        // z_N[j] = z_N[j]*Pi[j]; // Multiply by the inverse of the Hessian
        // z_N[j] = (z_N[j] > LB[j]) ? z_N[j] : LB[j]; // maximum between v and the lower bound
        // z_N[j] = (z_N[j] > UB[j]) ? UB[j] : z_N[j]; // minimum between v and the upper bound

    // }

}

/* compute_residual_equMPC_FISTA
*
*/

void compute_residual_vector_equMPC_FISTA(double res_vec[][nn_], double *z_0, double z[][nm_], double *b, double *xr, const double (*AB)[nm_]){

    // Compute the first nn_ elements
    for(unsigned int j = 0; j < nn_; j++){
            res_vec[0][j] = b[j] + z[0][j];
        for(unsigned int i = 0; i < mm_; i++){
            res_vec[0][j] = res_vec[0][j] - AB[j][i+nn_]*z_0[i];
        }
    }

    // Compute all the other elements except the last nn_
    for(unsigned int l = 1; l < NN_-1; l++){
        for(unsigned int j = 0; j < nn_; j++){
            res_vec[l][j] = z[l][j];
            for(unsigned int i = 0; i < nm_; i++){
                res_vec[l][j] = res_vec[l][j] - AB[j][i]*z[l-1][i];
            }
        }
    }

    // Compute the last nn_ elements
    for(unsigned int j = 0; j < nn_; j++){
        res_vec[NN_-1][j] = xr[j];
        for(unsigned int i = 0; i < nm_; i++){
            res_vec[NN_-1][j] = res_vec[NN_-1][j] - AB[j][i]*z[NN_-2][i];
        }
    }

}

#if TIME_VARYING  == 1
void solve_W_matrix_form(double mu[][nn_], double (*Alpha)[nn_][nn_], double (*Beta)[nn_][nn_]) {
#else
void solve_W_matrix_form(double mu[][nn_], const double (*Alpha)[nn_][nn_], const double (*Beta)[nn_][nn_]){
#endif

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

}

