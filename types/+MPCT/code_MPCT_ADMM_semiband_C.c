/**
 * ADMM solver for the MPCT formulation using Woodbury Matrix Identity
 *
 * ARGUMENTS:
 * The current system state is given in "x0_in". Pointer to array of size nn_.
 * The state reference is given in "xr_in". Pointer to array of size nn_.
 * The input reference is given in "ur_in". Pointer to array of size mm_.
 * The optimal control action is returned in "u_opt". Pointer to array of size mm_.
 * The number of iterations is returned in "k_in". Pointer to int.
 * The exit flag is returned in "e_flag". Pointer to int.
 *       1: Algorithm converged succesfully.
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

void MPCT_ADMM_semiband(double *x0_in, double *xr_in, double *ur_in, double *u_opt, int *k_in, int *e_flag, solution_MPCT *sol){

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

    // Initialize solver variables
    int done = 0;
    int k = 0; // Number of iterations
    double x0[nn_] = {0.0}; // Current system state
    double xr[nn_] = {0.0}; // State reference
    double ur[mm_] = {0.0}; // Input reference
    double z[(NN_+1)*nm_] = {0.0}; // Decision variable z
    double v[(NN_+1)*nm_] = {0.0}; // Decision variable v
    double v_old[(NN_+1)*nm_] = {0.0}; // Decision variable v in the previous iteration
    double lambda[(NN_+1)*nm_] = {0.0}; // Decision variable lambda
    double q[nm_] = {0.0}; // Linear term vector in the functional. Only non-zero elements are considered.
    double b[(NN_+2)*nn_] = {0.0}; // Independent term of the equlity constraint
    double xi[(NN_+1)*nm_] = {0.0}; // Used to solve the equality-constrained QP step
    double mu[(NN_+2)*nn_] = {0.0}; // Used to solve the equality-constrained QP step
    double z1_ac[(NN_+1)*nm_] = {0.0}; // Used to solve the equality-constrained QP step. Stores z1_a and z1_c.
    double z3_ac[(NN_+1)*nm_] = {0.0}; // Used to solve the equality-constrained QP step. Stores z3_a and z3_c.
    double z1_b[(NN_+2)*nn_] = {0.0}; // Used to solve the equality-constrained QP step
    double z3_b[(NN_+2)*nn_] = {0.0}; // Used to solve the equality-constrained QP step
    double z2[2*nm_] = {0.0}; // Used to solve the equality-constrained QP step. This one is used as z2_a, z2_b and z2_c.
    double p[(NN_+1)*nm_] = {0.0}; // Used to solve the equality-constrained QP
    double aux_0[(NN_+1)*nm_] = {0.0}; // Auxiliary vector used to solve the equality-constrained QP step.
    double aux_1[(NN_+2)*nn_] = {0.0}; // Auxiliary vector used to solve the equality-constrained QP step.
    double res_fixed_point; // Variable used to determine if a fixed point has been reached
    double res_primal_feas; // Variable used to determine if primal feasibility is satisfied
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

    // Compute b

    for(unsigned int i = 0 ; i< nn_ ; i++){
        b[i] = x0[i];
    }


    // Compute q

    for(unsigned int i = 0 ; i < nn_ ; i++){
        
        for(unsigned int j = 0 ; j < nn_ ; j++){

            q[i] -= T[i][j] * xr[j];

        }

    }

    for(unsigned int i = 0 ; i < mm_ ; i++){
        
        for(unsigned int j = 0 ; j < mm_ ; j++){

            q[nn_ + i] -= S[i][j] * ur[j];

        }

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

        k += 1;

        // Save the value of v
        memcpy(v_old, v, sizeof(double)*(NN_+1)*nm_);
        
        // Reset acumulator variables
        memset(z1_ac, 0, sizeof(double)*(NN_+1)*nm_);
        memset(z3_ac, 0, sizeof(double)*(NN_+1)*nm_);
        memset(z1_b, 0, sizeof(double)*(NN_+2)*nn_);
        memset(z3_b, 0, sizeof(double)*(NN_+2)*nn_);
        memset(z2, 0, sizeof(double)*2*nm_); 

        //********** Equality-constrained QP solve **********//
        // This problem updates z
        for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){

            #ifdef SCALAR_RHO
            p[i] = lambda[i] - rho * v[i];
            #else
            p[i] = lambda[i] - rho[i] * v[i];
            #endif

        }

        for (unsigned int i = 0 ; i < nm_ ; i++){

            p[i+NN_*nm_] += q[i];

        }


        /****** Compute xi from eq. (9a) using Alg. 2 from the article ******/

        solve_banded_diag_sys(Q_rho_i, R_rho_i, S_rho_i, T_rho_i, z1_ac, p); // Obtains z1_a

        // z2_a = M_hat * z1_a computed sparsely
        for (unsigned int i = 0 ; i < nn_ ; i++){

            for (unsigned int l = 0 ; l < NN_ ; l++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    z2[i] += M_hat_x[i][j]*z1_ac[l*nm_+j];
                    
                }
                
            }

            for (unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_hat_x[i][j+nn_]*z1_ac[NN_*nm_+j];
                
            }

        }

        for (unsigned int i = nn_ ; i < nm_ ; i++){

            for (unsigned int l = 0 ; l < NN_ ; l++){
            
                for (unsigned int j = 0 ; j < mm_ ; j++){
                
                    z2[i] += M_hat_u[i-nn_][j]*z1_ac[l*nm_+nn_+j];
                
                }

            }

            for (unsigned int j = 0 ; j < mm_ ; j++){
                
                z2[i] += M_hat_u[i-nn_][j+mm_]*z1_ac[NN_*nm_+nn_+j];
            
            }

        }

        for (unsigned int i = nm_ ; i < 2*nn_+mm_ ; i++){

            for (unsigned int l = 0 ; l < NN_ ; l++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    z2[i] += M_hat_x[i-mm_][j]*z1_ac[l*nm_+j];
                    
                }

            }

            for (unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_hat_x[i-mm_][j+nn_]*z1_ac[NN_*nm_+j];
                
            }

        }

        for (unsigned int i = nm_+nn_ ; i < 2*nm_ ; i++){

            for (unsigned int l = 0 ; l < NN_ ; l++){

                for (unsigned int j = 0 ; j < mm_ ; j++){

                    z2[i] += M_hat_u[i-2*nn_][j]*z1_ac[l*nm_+nn_+j];

                }

            }

            for (unsigned int j = 0 ; j < mm_ ; j++){
                    
                z2[i] += M_hat_u[i-2*nn_][j+mm_]*z1_ac[NN_*nm_+nn_+j];

            }

        }

        memset(aux_0, 0, sizeof(double)*(NN_+1)*nm_);

        // (U_hat * z2_a) computed sparsely
        for (unsigned int l = 0 ; l < NN_ ; l++){

            for (unsigned int i = l*nm_ ; i < l*nm_ + nn_ ; i++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    aux_0[i] -= Q[i-l*nm_][j] * z2[j];

                }

            }

            for (unsigned int  i = l*nm_+nn_ ; i < (l+1)*nm_ ; i++){

                for (unsigned int j = 0 ; j < mm_ ; j++){

                    aux_0[i] -= R[i-l*nm_-nn_][j] * z2[j+nn_];

                }

            }

        }

        for (unsigned int i = NN_*nm_ ; i < (NN_+1)*nm_ ; i++){

            aux_0[i] = z2[i-(NN_-1)*nm_];

        }
        // End of computation of (U_hat * z2_a)

        solve_banded_diag_sys(Q_rho_i, R_rho_i, S_rho_i, T_rho_i, z3_ac, aux_0); // Obtains z3_a

        // Computation of xi, which is the solution of eq. (9a)
        for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){

            xi[i] = z1_ac[i] - z3_ac[i];

        }
        // End of computation of xi


        /****** Compute mu from eq. (9b) using Alg. 2 from the article ******/
        
        // Sparse computation of -(G*xi+b)
        memset(aux_1, 0, sizeof(double)*(NN_+2)*nn_);

        for (unsigned int i = 0 ; i < nn_ ; i++){

            aux_1[i] = -(b[i]+xi[i]);

        }

        for(unsigned int l = 1 ; l <= NN_ ; l++){

            for (unsigned int i = l*nn_ ; i < (l+1)*nn_ ; i++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    if (i-l*nn_ == j){
                        aux_1[i] -= (A[i-l*nn_][j] * xi[(l-1)*nm_+j] - xi[l*nm_+j]);
                    }
                    else{
                        aux_1[i] -= A[i-l*nn_][j] * xi[(l-1)*nm_+j];
                    }

                }

                for (unsigned int j = 0 ; j < mm_ ; j++){
                    aux_1[i] -= B[i-l*nn_][j] * xi[(l-1)*nm_+nn_+j];
                }

            }

        }

        for (unsigned int i = (NN_+1)*nn_ ; i < (NN_+2)*nn_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                if (i-(NN_+1)*nn_ == j){
                    aux_1[i] -= (A[i-(NN_+1)*nn_][j]-1.0) * xi[NN_*nm_+j];
                }
                else{
                    aux_1[i] -= A[i-(NN_+1)*nn_][j] * xi[NN_*nm_+j];
                }

            }

            
            for (unsigned int j = 0 ; j < mm_ ; j++){
                aux_1[i] -= B[i-(NN_+1)*nn_][j] * xi[NN_*nm_+nn_+j];
            }

        }
        // End of computation of -(G*xi+b)

        solve_banded_Chol(Alpha, Beta, z1_b, aux_1); // Obtains z1_b

        memset(z2, 0, sizeof(double)*2*nm_);

        // Computation of z2_b
        for  (unsigned int i = 0 ; i < 2*nm_ ; i++){ 

            for(unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_tilde[i][j] * z1_b[j]; // M_tilde is dense, but it presents repetitions inside, so we use a shortened version of it instead

            }

            for (unsigned int l = 0 ; l < NN_-1 ; l++){
            
                for (unsigned int j = nn_ ; j < 2*nn_ ; j++){
                
                    z2[i] += M_tilde[i][j] * z1_b[l*nn_+j];

                }
                
            }

            for (unsigned int j = 2*nn_ ; j < 4*nn_ ; j++){

                z2[i] += M_tilde[i][j] * z1_b[(NN_-2)*nn_+j];

            }

        }
        // End of computation of z2_b

        memset(aux_1, 0, sizeof(double)*(NN_+2)*nn_);

        // Computation of (U_tilde*z2_b)
        for  (unsigned int i = 0 ; i < 2*nm_ ; i++){ 

            for(unsigned int j = 0 ; j < nn_ ; j++){

                aux_1[j] += U_tilde[j][i] * z2[i]; // U_tilde is dense, but it presents repetitions inside, so we use a shortened version of it instead

            }

            for (unsigned int l = 0 ; l < NN_-1 ; l++){
            
                for (unsigned int j = nn_ ; j < 2*nn_ ; j++){
                
                    aux_1[l*nn_+j] += U_tilde[j][i] * z2[i];

                }
                
            }

            for (unsigned int j = 2*nn_ ; j < 4*nn_ ; j++){

                aux_1[(NN_-2)*nn_+j] += U_tilde[j][i] * z2[i];

            }

        }
        // End of computation of (U_tilde*z2_b)

        solve_banded_Chol(Alpha, Beta, z3_b, aux_1); // Obtains z3_b

        // Computation of mu, which is the solution of eq. (9b)
        for (unsigned int i = 0 ; i < (NN_+2)*nn_ ; i++){ 

            mu[i] = z1_b[i] - z3_b[i];

        }
        // End of computation of mu

        
        /****** Compute z^{k+1} from eq. (9c) using ALg. 2 from the article ******/
        memset(aux_0, 0, sizeof(double)*(NN_+1)*nm_);

        // Sparse computation of -(G'*mu+p)
        for (unsigned int i = 0 ; i < nn_ ; i++){

            for (unsigned int j = 0; j < nn_ ; j++){

                if (i == j){
                    aux_0[i] -= A[j][i] * mu[nn_+j] + mu[j];
                }

                else{
                    aux_0[i] -= A[j][i] * mu[nn_+j];
                }

            }

            aux_0[i] -= p[i];

        }

        for (unsigned int i = nn_ ; i<nm_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                aux_0[i] -= B[j][i-nn_] * mu[nn_+j];

            }

            aux_0[i] -= p[i];

        }

        for (unsigned int l = 2 ; l<= NN_ ; l++){

            for (unsigned int i = (l-1)*nm_ ; i < l*nn_+(l-1)*mm_ ; i++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    if(i-(l-1)*nm_ == j){
                        aux_0[i] -= (A[j][i-(l-1)*nm_] * mu[l*nn_+j] - mu[j+(l-1)*nn_]);
                    }
                    else{
                        aux_0[i] -= A[j][i-(l-1)*nm_] * mu[l*nn_+j];
                    }

                }

                aux_0[i] -= p[i];

            }

            for (unsigned int i = l*nm_ - mm_ ; i < l*nm_ ; i++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    aux_0[i] -= B[j][i-l*nm_+mm_] * mu[l*nn_+j];

                }

                aux_0[i] -= p[i];

            }

        }

        for (unsigned int i = NN_*nm_ ; i < NN_*nm_ + nn_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                if (i-NN_*nm_ == j){
                    aux_0[i] -= ((A[j][i-NN_*nm_]-1.0) * mu[(NN_+1)*nn_+j] - mu[j+NN_*nn_]);
                }
                else{
                    aux_0[i] -= A[j][i-NN_*nm_] * mu[(NN_+1)*nn_+j];
                }

            }

            aux_0[i] -= p[i];

        }

        for (unsigned int i = NN_*nm_+nn_ ; i<(NN_+1)*nm_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                aux_0[i] -= B[j][i-NN_*nm_-nn_] * mu[(NN_+1)*nn_+j];

            }

            aux_0[i] -= p[i];

        }

        // End of computation of -(G'*mu+p)

        memset(z1_ac, 0, sizeof(double)*(NN_+1)*nm_);

        solve_banded_diag_sys(Q_rho_i, R_rho_i, S_rho_i, T_rho_i, z1_ac, aux_0); // Obtains z1_c

        memset(z2, 0, sizeof(double)*2*nm_);

        for (unsigned int i = 0 ; i < nn_ ; i++){

            for (unsigned int l = 0 ; l < NN_+1 ; l++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    if (l == NN_){
                        z2[i] += M_hat_x[i][j+nn_]*z1_ac[l*nm_+j];
                    }
                    else{
                        z2[i] += M_hat_x[i][j]*z1_ac[l*nm_+j];
                    }
                    
                }
                
            }

        }

        for (unsigned int i = nn_ ; i < nm_ ; i++){

            for (unsigned int l = 0 ; l < NN_+1 ; l++){
            
                for (unsigned int j = 0 ; j < mm_ ; j++){
                    if (l == NN_){
                        z2[i] += M_hat_u[i-nn_][j+mm_]*z1_ac[l*nm_+nn_+j];
                    }
                    else{
                        z2[i] += M_hat_u[i-nn_][j]*z1_ac[l*nm_+nn_+j];
                    }
                }

            }

        }

        for (unsigned int i = nm_ ; i < 2*nn_+mm_ ; i++){

            for (unsigned int l = 0 ; l < NN_+1 ; l++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    if(l == NN_){
                        z2[i] += M_hat_x[i-mm_][j+nn_]*z1_ac[l*nm_+j];
                    }
                    else{
                        z2[i] += M_hat_x[i-mm_][j]*z1_ac[l*nm_+j];
                    }

                    
                }

            }

        }

        for (unsigned int i = nm_+nn_ ; i < 2*nm_ ; i++){

            for (unsigned int l = 0 ; l < NN_+1 ; l++){

                for (unsigned int j = 0 ; j < mm_ ; j++){
                    
                    if (l == NN_){
                        z2[i] += M_hat_u[i-2*nn_][j+mm_]*z1_ac[l*nm_+nn_+j];
                    }
                    else{
                        z2[i] += M_hat_u[i-2*nn_][j]*z1_ac[l*nm_+nn_+j];
                    }

                }

            }

        }

        memset(aux_0, 0, sizeof(double)*(NN_+1)*nm_);

        // (U_hat * z2_c) computed sparsely
        for (unsigned int l = 0 ; l < NN_ ; l++){

            for (unsigned int i = l*nm_ ; i < l*nm_ + nn_ ; i++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    aux_0[i] -= Q[i-l*nm_][j] * z2[j];

                }

            }

            for (unsigned int  i = l*nm_+nn_ ; i < (l+1)*nm_ ; i++){

                for (unsigned int j = 0 ; j < mm_ ; j++){

                    aux_0[i] -= R[i-l*nm_-nn_][j] * z2[j+nn_];

                }

            }

        }

        for (unsigned int i = NN_*nm_ ; i < (NN_+1)*nm_ ; i++){

            aux_0[i] = z2[i-(NN_-1)*nm_];

        }
        // End of computation of (U_hat * z2_c)

        memset(z3_ac, 0, sizeof(double)*(NN_+1)*nm_);

        solve_banded_diag_sys(Q_rho_i, R_rho_i, S_rho_i, T_rho_i, z3_ac, aux_0); // Obtains z3_c

        // Computation of  z^{k+1}, which is the solution of eq. (9c)
        for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){

            z[i] = z1_ac[i] - z3_ac[i];

        }
        // End of computation of z^{k+1}

        //********** Inequality-constrained QP solve **********//
        // This problem updates v

        for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){

            #ifdef SCALAR_RHO
            v[i] = rho_i * lambda[i] + z[i];
            #else
            v[i] = rho_i[i] * lambda[i] + z[i];
            #endif

        }

        for (unsigned int i = 0 ; i < nn_ ; i++){

            v[i] = (v[i] > -inf) ? v[i] : -inf;
            v[i] = (v[i] < inf) ? v[i] : inf;
        
        }

        for (unsigned int i = nn_ ; i < nm_ ; i++){

            v[i] = (v[i] > LB[i]) ? v[i] : LB[i];
            v[i] = (v[i] < UB[i]) ? v[i] : UB[i];
        
        }

        for (unsigned int l = 1 ; l < NN_ ; l++){

            for (unsigned int i = l*nm_ ; i < (l+1)*nm_ ; i++){
                
                v[i] = (v[i] > LB[i-l*nm_]) ? v[i] : LB[i-l*nm_];
                v[i] = (v[i] < UB[i-l*nm_]) ? v[i] : UB[i-l*nm_];

            }

        }

        for (unsigned int i = NN_*nm_ ; i < NN_*nm_+nn_ ; i++){

            v[i] = (v[i] > LB[i-NN_*nm_]+eps_x) ? v[i] : LB[i-NN_*nm_]+eps_x;
            v[i] = (v[i] < UB[i-NN_*nm_]-eps_x) ? v[i] : UB[i-NN_*nm_]-eps_x;

        }

        for (unsigned int i = NN_*nm_+nn_ ; i < (NN_+1)*nm_ ; i++){

            v[i] = (v[i] > LB[i-NN_*nm_]+eps_u) ? v[i] : LB[i-NN_*nm_]+eps_u;
            v[i] = (v[i] < UB[i-NN_*nm_]-eps_u) ? v[i] : UB[i-NN_*nm_]-eps_u;

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
    *k_in = k;

    // Save solution into structure
    #ifdef DEBUG

    for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){
        
        sol->z[i] = z[i];
        sol->v[i] = v[i];
        sol->lambda[i] = lambda[i];
    
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
            
            y[(k)*nn_+i] = (d[(k)*nn_+i] - sum) * Beta[k][i][i]; // This is a division by the diagonal of Beta, but the diagonal of Beta is inverted, so we multiplity instead by the diagonal inverted

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

            z[(k-1)*nn_+i-1] = (y[(k-1)*nn_+i-1] - sum) * Beta[k-1][i-1][i-1]; // This is a division by the diagonal of Beta, but the diagonal of Beta is inverted, so we multiplity instead by the diagonal inverted

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