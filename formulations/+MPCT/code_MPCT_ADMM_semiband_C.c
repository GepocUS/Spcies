/**
 * ADMM solver for the MPCT formulation using Woodbury Matrix Identity. Features two different modes:
 * > SOFT_CONSTRAINTS == 0: MPCT formulation with box constraints in states and inputs
 * > SOFT_CONSTRAINTS == 1: MPCT formulation with hard box constraints in u_0, and "softened" box constraints
 * in the rest of inputs, states, and also in constrained outputs, i.e., y_c = C_c*x_i+D_c*u_i.
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
 * Computation times are also returned in the structure sol.
 * 
 */

void MPCT_ADMM_semiband(double *x0_in, double *xr_in, double *ur_in, double *u_opt, int *k_in, int *e_flag, sol_$INSERT_NAME$ *sol){

    #if MEASURE_TIME == 1

    #if WIN32
    static LARGE_INTEGER start, post_update, post_solve, post_polish;
    #else // If Linux
    struct timespec start, post_update, post_solve, post_polish;
    #endif

    read_time(&start);

    #endif

    // Initialize solver variables
    int done = 0;
    int k = 0; // Number of iterations
    double x0[nn_] = {0.0}; // Current system state
    double xr[nn_] = {0.0}; // State reference
    double ur[mm_] = {0.0}; // Input reference
    double z[(NN_+1)*nm_] = {0.0}; // Decision variable z
    #if CONSTRAINED_OUTPUT == 0
    double v[(NN_+1)*nm_] = {0.0}; // Decision variable v
    double v_old[(NN_+1)*nm_] = {0.0}; // Decision variable v in the previous iteration
    double lambda[(NN_+1)*nm_] = {0.0}; // Decision variable lambda
    #else
    double v[(NN_+1)*nmp_] = {0.0}; // Decision variable v
    double v_old[(NN_+1)*nmp_] = {0.0}; // Decision variable v in the previous iteration
    double lambda[(NN_+1)*nmp_] = {0.0}; // Decision variable lambda
    double C_tilde_z_v[(NN_+1)*nmp_] = {0.0}; // Auxiliary vector for storing C*z-v
    #endif
    #if SOFT_CONSTRAINTS
    double v_aux1 = 0.0; // Used for computation of v when SOFT_CONSTRAINTS == 1
    // double v_aux2 = 0.0; // Used for computation of v when SOFT_CONSTRAINTS == 1
    double v_aux3 = 0.0; // Used for computation of v when SOFT_CONSTRAINTS == 1
    #endif
    double q[nm_] = {0.0}; // Linear term vector in the functional. Only non-zero elements are considered.
    double xi[(NN_+1)*nm_] = {0.0}; // Used to solve the equality-constrained QP step
    double mu[(NN_+2)*nn_] = {0.0}; // Used to solve the equality-constrained QP step
    double z3_ac[(NN_+1)*nm_] = {0.0}; // Used to solve the equality-constrained QP step. Stores z3_a and z3_c.
    double z2[2*nm_] = {0.0}; // Used to solve the equality-constrained QP step. This one is used as z2_a, z2_b and z2_c.
    double p[(NN_+1)*nm_] = {0.0}; // Used to solve the equality-constrained QP
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
    read_time(&post_update);
    get_elapsed_time(&sol->update_time, &post_update, &start);
    #endif

    // Algorithm
    while(done == 0){

        k += 1;

        // Save the value of v
        #if CONSTRAINED_OUTPUT == 0
        memcpy(v_old, v, sizeof(double)*(NN_+1)*nm_);
        #else // CONSTRAINED_OUTPUT == 1
        memcpy(v_old, v, sizeof(double)*(NN_+1)*nmp_);
        #endif        
        // Reset acumulator variables
        memset(xi, 0, sizeof(double)*(NN_+1)*nm_);
        memset(z3_ac, 0, sizeof(double)*(NN_+1)*nm_);
        memset(z2, 0, sizeof(double)*2*nm_); 

        //********** Equality-constrained QP solve **********//
        // This problem updates z
        #if CONSTRAINED_OUTPUT == 0
        for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){

            #ifdef SCALAR_RHO
            p[i] = lambda[i] - rho * v[i];
            #else
            p[i] = lambda[i] - rho[i] * v[i];
            #endif

        }

        #else // CONSTRAINED_OUTPUT == 1
        // Compute p = q+C_tilde'*(lambda-rho*v);
        for (unsigned int l = 0 ; l < NN_+1 ; l++){
        
            for (unsigned int i = 0 ; i < nm_ ; i++){

                #ifdef SCALAR_RHO

                p[l*nm_+i] = lambda[l*nmp_+i] - rho * v[l*nmp_+i];
            
                for (unsigned int j = 0; j < pp_ ; j++){
                
                    p[l*nm_+i] += CD[j][i] * (lambda[l*nmp_+nm_+j] - rho*v[l*nmp_+nm_+j]);

                }

                #else

                p[l*nm_+i] = lambda[l*nmp_+i] - rho[l*nmp_+i] * v[l*nmp_+i];
            
                for (unsigned int j = 0; j < pp_ ; j++){
                
                    p[l*nm_+i] += CD[j][i] * (lambda[l*nmp_+nm_+j] - rho[l*nmp_+nm_+j]*v[l*nmp_+nm_+j]);

                }

                #endif

            }

        }

        #endif

        for (unsigned int i = 0 ; i < nm_ ; i++){

            p[i+NN_*nm_] += q[i];

        }

        /****** Compute xi from eq. (9a) using Alg. 2 from the article ******/

        solve_banded_QRST_sys(Q_rho_i, R_rho_i, S_rho_i, T_rho_i, xi, p); // Obtains z1_a, stored in xi to save memory

        // z2_a = M_hat * z1_a computed sparsely
        
        for (unsigned int l = 0 ; l < NN_ ; l++){
            
            for (unsigned int i = 0 ; i < nn_ ; i++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    z2[i] += M_hat_x1[i][j]*xi[l*nm_+j];
                    
                }
                
            }

        }
        
        for (unsigned int i = 0 ; i < nn_ ; i++){
            
            for (unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_hat_x2[i][j]*xi[NN_*nm_+j];
                
            }

        }

        for (unsigned int l = 0 ; l < NN_ ; l++){

            for (unsigned int i = nn_ ; i < nm_ ; i++){
            
                for (unsigned int j = 0 ; j < mm_ ; j++){
                
                    z2[i] += M_hat_u1[i-nn_][j]*xi[l*nm_+nn_+j];
                
                }

            }
        
        }

        for (unsigned int i = nn_ ; i < nm_ ; i++){

            for (unsigned int j = 0 ; j < mm_ ; j++){
                
                z2[i] += M_hat_u2[i-nn_][j]*xi[NN_*nm_+nn_+j];
            
            }

        }

        for (unsigned int l = 0 ; l < NN_ ; l++){

            for (unsigned int i = nm_ ; i < nm_+nn_ ; i++){
            
                for (unsigned int j = 0 ; j < nn_ ; j++){

                    z2[i] += M_hat_x1[i-mm_][j]*xi[l*nm_+j];
                    
                }

            }

        }

        for (unsigned int i = nm_ ; i < nm_+nn_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_hat_x2[i-mm_][j]*xi[NN_*nm_+j];
                
            }

        }

        for (unsigned int l = 0 ; l < NN_ ; l++){

            for (unsigned int i = nm_+nn_ ; i < 2*nm_ ; i++){

                for (unsigned int j = 0 ; j < mm_ ; j++){

                    z2[i] += M_hat_u1[i-2*nn_][j]*xi[l*nm_+nn_+j];

                }

            }

        }

        for (unsigned int i = nm_+nn_ ; i < 2*nm_ ; i++){

            for (unsigned int j = 0 ; j < mm_ ; j++){
                    
                z2[i] += M_hat_u2[i-2*nn_][j]*xi[NN_*nm_+nn_+j];

            }

        }

        #if CONSTRAINED_OUTPUT == 0

        memset(v, 0, sizeof(double)*(NN_+1)*nm_);

        #else // CONSTRAINED_OUTPUT == 1

        memset(v, 0, sizeof(double)*(NN_+1)*nmp_);

        #endif

        // (U_hat * z2_a) computed sparsely, stored in v to save memory
        for (unsigned int l = 0 ; l < NN_ ; l++){

            for (unsigned int i = l*nm_ ; i < l*nm_ + nn_ ; i++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    v[i] -= Q[i-l*nm_][j] * z2[j];

                }

            }

            for (unsigned int  i = l*nm_+nn_ ; i < (l+1)*nm_ ; i++){

                for (unsigned int j = 0 ; j < mm_ ; j++){

                    v[i] -= R[i-l*nm_-nn_][j] * z2[j+nn_];

                }

            }

        }

        for (unsigned int i = NN_*nm_ ; i < (NN_+1)*nm_ ; i++){

            v[i] = z2[i-(NN_-1)*nm_];

        }
        // End of computation of (U_hat * z2_a), stored in v

        solve_banded_QRST_sys(Q_rho_i, R_rho_i, S_rho_i, T_rho_i, z3_ac, v); // Obtains z3_a

        
        // Computation of xi, which is the solution of eq. (9a)
        for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){

            xi[i] -= z3_ac[i]; // xi[i] = z1_ac[i] - z3_ac[i];

        }
        // End of computation of xi


        /****** Compute mu from eq. (9b) using Alg. 2 from the article ******/
        
        // Sparse computation of -(G*xi+b), stored in mu to save memory
        memset(mu, 0, sizeof(double)*(NN_+2)*nn_);

        for (unsigned int i = 0 ; i < nn_ ; i++){

            mu[i] = -(x0[i] + xi[i]); // -(b[i]+xi[i]);

        }

        for(unsigned int l = 1 ; l <= NN_ ; l++){

            for (unsigned int i = l*nn_ ; i < (l+1)*nn_ ; i++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    mu[i] -= A[i-l*nn_][j] * xi[(l-1)*nm_+j];

                }

                mu[i] += xi[l*nm_+i-l*nn_];

                for (unsigned int j = 0 ; j < mm_ ; j++){

                    mu[i] -= B[i-l*nn_][j] * xi[(l-1)*nm_+nn_+j];
                    
                }

            }

        }

        for (unsigned int i = (NN_+1)*nn_ ; i < (NN_+2)*nn_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                mu[i] -= A[i-(NN_+1)*nn_][j] * xi[NN_*nm_+j];

            }

            mu[i] += xi[NN_*nm_+i-(NN_+1)*nn_];

            for (unsigned int j = 0 ; j < mm_ ; j++){

                mu[i] -= B[i-(NN_+1)*nn_][j] * xi[NN_*nm_+nn_+j];

            }

        }
        // End of computation of -(G*xi+b)

        solve_banded_Chol(Alpha, Beta, mu); // Obtains z1_b. We use mu to store the result. Note that mu contained the independent term vector -(G*xi+b) before calling this function.

        memset(z2, 0, sizeof(double)*2*nm_);

        // Computation of z2_b
        #ifdef SCALAR_RHO
        for  (unsigned int i = 0 ; i < 2*nm_ ; i++){ 

            for(unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_tilde[i][j] * mu[j]; // z2[i] += M_tilde[i][j] * z1_b[j]. M_tilde is dense, but it presents repetitions inside if rho is scalar, so we use a shortened version of it instead

            }

            for (unsigned int l = 0 ; l < NN_-1 ; l++){
            
                for (unsigned int j = nn_ ; j < 2*nn_ ; j++){
                
                    z2[i] += M_tilde[i][j] * mu[l*nn_+j]; // z2[i] += M_tilde[i][j] * z1_b[l*nn_+j];

                }
                
            }

            for (unsigned int j = 2*nn_ ; j < 4*nn_ ; j++){

                z2[i] += M_tilde[i][j] * mu[(NN_-2)*nn_+j]; // z2[i] += M_tilde[i][j] * z1_b[(NN_-2)*nn_+j];

            }

        }
        
        #else
        
        for  (unsigned int i = 0 ; i < 2*nm_ ; i++){ 

            for(unsigned int j = 0 ; j < (NN_+2)*nn_ ; j++){

                z2[i] += M_tilde[i][j] * mu[j]; // z2[i] += M_tilde[i][j] * z1_b[j]. M_tilde is dense. It does not have repetitions inside when rho is a vector, so we use the full matrix

            }

        }

        #endif
        // End of computation of z2_b

        #if CONSTRAINED_OUTPUT == 0

        memset(v, 0, sizeof(double)*(NN_+1)*nm_);

        #else // CONSTRAINED_OUTPUT == 1

        memset(v, 0, sizeof(double)*(NN_+1)*nmp_);

        #endif

        // Computation of (U_tilde*z2_b), stored in v to save memory
        #ifdef SCALAR_RHO

        for (unsigned int i = 0 ; i < 2*nm_ ; i++){ 

            for(unsigned int j = 0 ; j < nn_ ; j++){

                v[j] += U_tilde[j][i] * z2[i]; // U_tilde is dense, but it presents repetitions inside if rho is scalar, so we use a shortened version of it instead

            }

            for (unsigned int l = 0 ; l < NN_-1 ; l++){
            
                for (unsigned int j = nn_ ; j < 2*nn_ ; j++){
                
                    v[l*nn_+j] += U_tilde[j][i] * z2[i]; // U_tilde is dense. It does not have repetitions inside when rho is a vector, so we use the full matrix

                }
                
            }

            for (unsigned int j = 2*nn_ ; j < 4*nn_ ; j++){

                v[(NN_-2)*nn_+j] += U_tilde[j][i] * z2[i];

            }

        }

        #else

        for (unsigned int i = 0 ; i < 2*nm_ ; i++){

            for (unsigned int j = 0 ; j < (NN_+2)*nn_ ; j++){
                
                v[j] += U_tilde[j][i] * z2[i];

            }

        }

        #endif
        // End of computation of (U_tilde*z2_b)

        solve_banded_Chol(Alpha, Beta, v); // Obtains z3_b, which is stored in v to save memory

        // Computation of mu, which is the solution of eq. (9b)
        for (unsigned int i = 0 ; i < (NN_+2)*nn_ ; i++){ 

            mu[i] -= v[i]; // mu[i] = z1_b[i] - z3_b[i];

        }
        // End of computation of mu
        
        
        /****** Compute z^{k+1} from eq. (9c) using ALg. 2 from the article ******/

        // Sparse computation of -(G'*mu+p). We store it in variable p.

        for (unsigned int i = 0 ; i < nn_ ; i++){

            p[i] = -(p[i] + mu[i]);

            for (unsigned int j = 0; j < nn_ ; j++){

                p[i] -= A[j][i] * mu[nn_+j];

            }

        }

        for (unsigned int i = nn_ ; i < nm_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                p[i] += B[j][i-nn_] * mu[nn_+j];

            }

            p[i] = -p[i];

        }

        for (unsigned int l = 2 ; l <= NN_ ; l++){

            for (unsigned int i = (l-1)*nm_ ; i < l*nn_+(l-1)*mm_ ; i++){

                p[i] = -(p[i] - mu[i-(l-1)*mm_]);

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    p[i] -= A[j][i-(l-1)*nm_] * mu[l*nn_+j];

                }

            }

            for (unsigned int i = l*nm_ - mm_ ; i < l*nm_ ; i++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    p[i] += B[j][i-l*nm_+mm_] * mu[l*nn_+j];

                }

                p[i] = -p[i];

            }

        }

        for (unsigned int i = NN_*nm_ ; i < NN_*nm_ + nn_ ; i++){

            p[i] = -p[i] + (mu[i+nn_-NN_*mm_] + mu[i-NN_*mm_]);

            for (unsigned int j = 0 ; j < nn_ ; j++){

                p[i] -= A[j][i-NN_*nm_] * mu[(NN_+1)*nn_+j];

            }

        }

        for (unsigned int i = NN_*nm_+nn_ ; i<(NN_+1)*nm_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                p[i] += B[j][i-NN_*nm_-nn_] * mu[(NN_+1)*nn_+j];

            }

            p[i] = -p[i];

        }

        // End of computation of -(G'*mu+p), which is stored in p.

        memset(z, 0, sizeof(double)*(NN_+1)*nm_);

        solve_banded_QRST_sys(Q_rho_i, R_rho_i, S_rho_i, T_rho_i, z, p); // Obtains z1_c, stored in z to save memory

        memset(z2, 0, sizeof(double)*2*nm_);


        for (unsigned int l = 0 ; l < NN_ ; l++){

            for (unsigned int i = 0 ; i < nn_ ; i++){

                for (unsigned int j = 0 ; j < nn_ ; j++){
                        
                    z2[i] += M_hat_x1[i][j]*z[l*nm_+j];
                    
                }
                
            }
        
        }

        for (unsigned int i = 0 ; i < nn_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_hat_x2[i][j]*z[NN_*nm_+j];

            }

        }

        for (unsigned int l = 0 ; l < NN_ ; l++){

            for (unsigned int i = nn_ ; i < nm_ ; i++){
            
                for (unsigned int j = 0 ; j < mm_ ; j++){
               
                    z2[i] += M_hat_u1[i-nn_][j]*z[l*nm_+nn_+j];

                }

            }

        }

        for (unsigned int i = nn_ ; i < nm_ ; i++){

            for (unsigned int j = 0 ; j < mm_ ; j++){

                z2[i] += M_hat_u2[i-nn_][j]*z[NN_*nm_+nn_+j];
            
            }

        }

        for (unsigned int l = 0 ; l < NN_ ; l++){

            for (unsigned int i = nm_ ; i < nm_+nn_ ; i++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    z2[i] += M_hat_x1[i-mm_][j]*z[l*nm_+j];
                    
                }

            }

        }

        for (unsigned int i = nm_ ; i < nm_+nn_ ; i++){

            for (unsigned int j = 0 ; j < nn_ ; j++){

                z2[i] += M_hat_x2[i-mm_][j]*z[NN_*nm_+j];
                
            }

        }

        for (unsigned int l = 0 ; l < NN_ ; l++){
            
            for (unsigned int i = nm_+nn_ ; i < 2*nm_ ; i++){

                for (unsigned int j = 0 ; j < mm_ ; j++){
                    
                    z2[i] += M_hat_u1[i-2*nn_][j]*z[l*nm_+nn_+j];

                }

            }

        }

        for (unsigned int i = nm_+nn_ ; i < 2*nm_ ; i++){

            for (unsigned int j = 0 ; j < mm_ ; j++){
                    
                z2[i] += M_hat_u2[i-2*nn_][j]*z[NN_*nm_+nn_+j];
            
            }

        }

        #if CONSTRAINED_OUTPUT == 0

        memset(v, 0, sizeof(double)*(NN_+1)*nm_);

        #else // CONSTRAINED_OUTPUT == 1

        memset(v, 0, sizeof(double)*(NN_+1)*nmp_);

        #endif

        // (U_hat * z2_c) computed sparsely, stored in v to save memory
        for (unsigned int l = 0 ; l < NN_ ; l++){

            for (unsigned int i = l*nm_ ; i < l*nm_ + nn_ ; i++){

                for (unsigned int j = 0 ; j < nn_ ; j++){

                    v[i] -= Q[i-l*nm_][j] * z2[j];

                }

            }

            for (unsigned int  i = l*nm_+nn_ ; i < (l+1)*nm_ ; i++){

                for (unsigned int j = 0 ; j < mm_ ; j++){

                    v[i] -= R[i-l*nm_-nn_][j] * z2[j+nn_];

                }

            }

        }

        for (unsigned int i = NN_*nm_ ; i < (NN_+1)*nm_ ; i++){

            v[i] = z2[i-(NN_-1)*nm_];

        }
        // End of computation of (U_hat * z2_c), stored in v

        memset(z3_ac, 0, sizeof(double)*(NN_+1)*nm_);

        solve_banded_QRST_sys(Q_rho_i, R_rho_i, S_rho_i, T_rho_i, z3_ac, v); // Obtains z3_c


        // Computation of  z^{k+1}, which is the solution of eq. (9c)
        for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){

            z[i] -= z3_ac[i];

        }
        // End of computation of z^{k+1}

        //********** Inequality-constrained QP solve **********//
        // This problem updates v

        #if CONSTRAINED_OUTPUT == 0

        for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){ // Computation of v = lambda/rho + z

            #ifdef SCALAR_RHO
            v[i] = rho_i * lambda[i] + z[i];
            #else
            v[i] = rho_i[i] * lambda[i] + z[i];
            #endif

        }

        // x_0 unconstrained
        for (unsigned int i = 0 ; i < nn_ ; i++){

            v[i] = (v[i] > -inf) ? v[i] : -inf;
            v[i] = (v[i] < inf) ? v[i] : inf;
        
        }

        // u_0 hard-constrained
        for (unsigned int i = nn_ ; i < nm_ ; i++){

            v[i] = (v[i] > LB[i]) ? v[i] : LB[i];
            v[i] = (v[i] < UB[i]) ? v[i] : UB[i];
        
        }

            #if SOFT_CONSTRAINTS == 0 // && CONSTRAINED_OUTPUT == 0
    
            // The rest of v is hard-constrained
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
    
            #else // CONSTRAINED_OUTPUT == 0 && SOFT_CONSTRAINTS == 1
    
            // The rest of v is soft-constrained
            for (unsigned int l = 1 ; l < NN_+1 ; l++){
    
                for (unsigned int i = l*nm_ ; i < (l+1)*nm_ ; i++){
                    
                    // v_aux2 = v[i];
                    #ifdef SCALAR_RHO
                    v_aux1 = v[i] + beta_rho_i;
                    v_aux3 = v[i] - beta_rho_i;
                    #else
                    v_aux1 = v[i] + beta_rho_i[i];
                    v_aux3 = v[i] - beta_rho_i[i];
                    #endif
                    
                    if (v_aux1 <= LB[i-l*nm_]){
                        v[i] = v_aux1;
                    }
                    // else if (v_aux2 >= LB[i-l*nmp_] && v_aux2 <= UB[i-l*nmp_]){
                    //     v[i] = v_aux2;
                    // }
                    else if(v_aux3 >= UB[i-l*nm_]){
                        v[i] = v_aux3;
                    }
                    else if (v[i] > UB[i-l*nm_]){ // else if (v_aux2 > UB[i-l*nmp_]){
                        v[i] = UB[i-l*nm_];
                    }
                    else if (v[i] < LB[i-l*nm_]){ // else if (v_aux2 < LB[i-l*nmp_]){
                        v[i] = LB[i-l*nm_];
                    }
    
                }
    
            }
            
    
            #endif

        #else // CONSTRAINED_OUTPUT == 1

            for (unsigned int l = 0 ; l < NN_+1 ; l++){ // Computation of v = lambda/rho + C_tilde*z
            
                for (unsigned int i = 0 ; i < nm_ ; i++){
    
                    #ifdef SCALAR_RHO
                    v[l*nmp_+i] = rho_i * lambda[l*nmp_+i] + z[l*nm_+i];
                    #else
                    v[l*nmp_+i] = rho_i[l*nmp_+i] * lambda[l*nmp_+i] + z[l*nm_+i];
                    #endif
    
                }
    
                for (unsigned int i = 0 ; i < pp_ ; i++){
    
                    #ifdef SCALAR_RHO
                    v[l*nmp_+nm_+i] = rho_i * lambda[l*nmp_+nm_+i];
                    #else
                    v[l*nmp_+nm_+i] = rho_i[l*nmp_+nm_+i] * lambda[l*nmp_+nm_+i];
                    #endif
    
                    for (unsigned int j = 0 ; j < nm_ ; j++){
                    
                        v[l*nmp_+nm_+i] += CD[i][j] * z[l*nm_+j];
        
                    }
                
                }
    
            }

            // x_0 unconstrained
            for (unsigned int i = 0 ; i < nn_ ; i++){
    
                v[i] = (v[i] > -inf) ? v[i] : -inf;
                v[i] = (v[i] < inf) ? v[i] : inf;
            
            }
            
            // u_0 hard-constrained
            for (unsigned int i = nn_ ; i < nm_ ; i++){
    
                v[i] = (v[i] > LB[i]) ? v[i] : LB[i];
                v[i] = (v[i] < UB[i]) ? v[i] : UB[i];
            
            }
    

            #if SOFT_CONSTRAINTS == 0 // CONSTRAINED_OUTPUT == 1 && SOFT_CONSTRAINTS == 0
                      
            // y_0 hard-constrained
            for (unsigned int i = nm_ ; i<nmp_ ; i++){

                v[i] = (v[i] > LB[i]) ? v[i] : LB[i];
                v[i] = (v[i] < UB[i]) ? v[i] : UB[i];

            }

            // The rest of the vector hard-constrained
            for (unsigned int l = 1 ; l < NN_ ; l++){
    
                for (unsigned int i = l*nmp_ ; i < (l+1)*nmp_ ; i++){
                    
                    v[i] = (v[i] > LB[i-l*nmp_]) ? v[i] : LB[i-l*nmp_];
                    v[i] = (v[i] < UB[i-l*nmp_]) ? v[i] : UB[i-l*nmp_];
    
                }
    
            }

            for (unsigned int i = NN_*nmp_ ; i < NN_*nmp_+nn_ ; i++){

                v[i] = (v[i] > LB[i-NN_*nmp_]+eps_x) ? v[i] : LB[i-NN_*nmp_]+eps_x;
                v[i] = (v[i] < UB[i-NN_*nmp_]-eps_x) ? v[i] : UB[i-NN_*nmp_]-eps_x;

            }

            for (unsigned int i = NN_*nmp_+nn_ ; i < NN_*nmp_+nm_ ; i++){

                v[i] = (v[i] > LB[i-NN_*nmp_]+eps_u) ? v[i] : LB[i-NN_*nmp_]+eps_u;
                v[i] = (v[i] < UB[i-NN_*nmp_]-eps_u) ? v[i] : UB[i-NN_*nmp_]-eps_u;

            }

            for (unsigned int i = NN_*nmp_+nm_ ; i < (NN_+1)*nmp_ ; i++){

                v[i] = (v[i] > LB[i-NN_*nmp_]+eps_y) ? v[i] : LB[i-NN_*nmp_]+eps_y;
                v[i] = (v[i] < UB[i-NN_*nmp_]-eps_y) ? v[i] : UB[i-NN_*nmp_]-eps_y;

            }
    
    
            #else // CONSTRAINED_OUTPUT == 1 && SOFT_CONSTRAINTS == 1
    
            // y_0 soft-constrained
            for (unsigned int i = nm_ ; i<nmp_ ; i++){
                
                // v_aux2 = v[i];
                #ifdef SCALAR_RHO
                v_aux1 = v[i] + beta_rho_i;
                v_aux3 = v[i] - beta_rho_i;
                #else
                v_aux1 = v[i] + beta_rho_i[i];
                v_aux3 = v[i] - beta_rho_i[i];
                #endif
    
                
                if (v_aux1 <= LB[i]){
                    v[i] = v_aux1;
                }
                // else if (v_aux2 >= LB[i] && v_aux2 <= UB[i]){
                //     v[i] = v_aux2;
                // }
                else if(v_aux3 >= UB[i]){
                    v[i] = v_aux3;
                }
                else if (v[i] > UB[i]){ // else if (v_aux2 > UB[i]){
                    v[i] = UB[i];
                }
                else if (v[i] < LB[i]){ // else if (v_aux2 < LB[i]){
                    v[i] = LB[i];
                }
    
            }
    
            // The rest of v is soft-constrained
            for (unsigned int l = 1 ; l < NN_+1 ; l++){
    
                for (unsigned int i = l*nmp_ ; i < (l+1)*nmp_ ; i++){
                    
                    // v_aux2 = v[i];
                    #ifdef SCALAR_RHO
                    v_aux1 = v[i] + beta_rho_i;
                    v_aux3 = v[i] - beta_rho_i;
                    #else
                    v_aux1 = v[i] + beta_rho_i[i];
                    v_aux3 = v[i] - beta_rho_i[i];
                    #endif
                    
                    if (v_aux1 <= LB[i-l*nmp_]){
                        v[i] = v_aux1;
                    }
                    // else if (v_aux2 >= LB[i-l*nmp_] && v_aux2 <= UB[i-l*nmp_]){
                    //     v[i] = v_aux2;
                    // }
                    else if(v_aux3 >= UB[i-l*nmp_]){
                        v[i] = v_aux3;
                    }
                    else if (v[i] > UB[i-l*nmp_]){ // else if (v_aux2 > UB[i-l*nmp_]){
                        v[i] = UB[i-l*nmp_];
                    }
                    else if (v[i] < LB[i-l*nmp_]){ // else if (v_aux2 < LB[i-l*nmp_]){
                        v[i] = LB[i-l*nmp_];
                    }
    
                }
    
            }
    
    
            #endif


        #endif

        //********** Update dual variables **********//

        #if CONSTRAINED_OUTPUT == 0
        for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){ // Computation of lambda^{k+1}
            
            #ifdef SCALAR_RHO
            lambda[i] += rho * (z[i] - v[i]);
            #else
            lambda[i] += rho[i] * (z[i] - v[i]);
            #endif

        }

        #else //CONSTRAINED_OUTPUT == 1

        memset(C_tilde_z_v, 0, sizeof(double)*(NN_+1)*nmp_);
            
        for (unsigned int l = 0 ; l < NN_+1 ; l++){ // Computation of C_tilde*z-v and lambda^{k+1} = lambda^k + rho * (C_tilde*z - v)
        
            for (unsigned int i = 0 ; i < nm_ ; i++){
            
                C_tilde_z_v[l*nmp_+i] = z[l*nm_+i];

                C_tilde_z_v[l*nmp_+i] -= v[l*nmp_+i];

                #ifdef SCALAR_RHO
                lambda[l*nmp_+i] += rho * (C_tilde_z_v[l*nmp_+i]);
                #else
                lambda[l*nmp_+i] += rho[l*nmp_+i] * (C_tilde_z_v[l*nmp_+i]);
                #endif

            }


            for (unsigned int i = 0 ; i < pp_ ; i++){

                for (unsigned int j = 0 ; j < nm_ ; j++){
                
                    C_tilde_z_v[l*nmp_+nm_+i] += CD[i][j] * z[l*nm_+j];
    
                }

                C_tilde_z_v[l*nmp_+nm_+i] -= v[l*nmp_+nm_+i];

                #ifdef SCALAR_RHO
                lambda[l*nmp_+nm_+i] += rho * (C_tilde_z_v[l*nmp_+nm_+i]);
                #else
                lambda[l*nmp_+nm_+i] += rho[l*nmp_+nm_+i] * (C_tilde_z_v[l*nmp_+nm_+i]);
                #endif
            
            }



        }

        #endif

        // Compute the residuals

        res_flag = 0; // Reset the residual flag

        #if CONSTRAINED_OUTPUT == 0

        for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){
            
            res_fixed_point = v[i] - v_old[i];
            res_primal_feas = z[i] - v[i];
            // Obtain absolute values
            res_fixed_point = (res_fixed_point > 0.0) ? res_fixed_point : -res_fixed_point;
            res_primal_feas = (res_primal_feas > 0.0) ? res_primal_feas : -res_primal_feas;
            
            if (res_fixed_point > tol_d || res_primal_feas > tol_p){

                res_flag = 1;
                break;

            }

        }

        #else // CONSTRAINED_OUTPUT == 1

        for (unsigned int i = 0 ; i < (NN_+1)*nmp_ ; i++){
        
            res_fixed_point = v[i] - v_old[i];
            res_primal_feas = C_tilde_z_v[i];
            // Obtain absolute values
            res_fixed_point = (res_fixed_point > 0.0) ? res_fixed_point : -res_fixed_point;
            res_primal_feas = (res_primal_feas > 0.0) ? res_primal_feas : -res_primal_feas;
            
            if (res_fixed_point > tol_d || res_primal_feas > tol_p){

                res_flag = 1;
                break;

            }

        }

        #endif

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
    read_time(&post_solve);
    get_elapsed_time(&sol->solve_time, &post_solve, &post_update);
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

    #if CONSTRAINED_OUTPUT == 0

    for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){
        
        sol->z[i] = z[i];
        sol->v[i] = v[i];
        sol->lambda[i] = lambda[i];
    
    }

    #else // CONSTRAINED_OUTPUT == 1

    for (unsigned int i = 0 ; i < (NN_+1)*nm_ ; i++){
        
        sol->z[i] = z[i];
    
    }

    for (unsigned int i = 0 ; i < (NN_+1)*nmp_ ; i++){
        
        sol->v[i] = v[i];
        sol->lambda[i] = lambda[i];
    
    }

    #endif

    #endif

    // Measure time
    #if MEASURE_TIME == 1
    read_time(&post_polish);
    get_elapsed_time(&sol->polish_time, &post_polish, &post_solve);
    get_elapsed_time(&sol->run_time, &post_polish, &start);
    #endif

}

void solve_banded_Chol(const double (*Alpha)[nn_][nn_], const double (*Beta)[nn_][nn_], double *d){

    // We are using the independent term vector "d" to return the solution vector "z" so as to save memory

    // Forward substitution

    for (unsigned int i = 0 ; i < nn_ ; i++){

        for(unsigned int p = 0 ; p < i ; p++){

            d[i] -= Beta[0][p][i] * d[p]; 
        
        }
        
        d[i] *= Beta[0][i][i]; // This is a division by the diagonal of Beta, but the diagonal of Beta is inverted, so we multiply instead by the diagonal inverted

    }

    for (unsigned int k=1 ; k < NN_+2 ; k++){
        
        for (unsigned int i = 0 ; i < nn_ ; i++){

            for(unsigned int p = 0 ; p < i ; p++){

                d[(k)*nn_+i] -= Beta[k][p][i] * d[(k)*nn_+p]; 
            
            }

            for(unsigned int p = 0; p < nn_ ; p++){

                d[(k)*nn_+i] -= Alpha[k-1][p][i] * d[(k-1)*nn_+p];

            }
            
            d[(k)*nn_+i] *= Beta[k][i][i]; // This is a division by the diagonal of Beta, but the diagonal of Beta is inverted, so we multiply instead by the diagonal inverted

        }

    }

    // Backward substitution

    for(unsigned int i = nn_ ; i > 0 ; i--){

        for(unsigned int p = i+1 ; p <= nn_ ; p++){

            d[(NN_+1)*nn_+i-1] -= Beta[NN_+1][i-1][p-1] * d[(NN_+1)*nn_+(p-1)];

        }

        d[(NN_+1)*nn_+i-1] *= Beta[NN_+1][i-1][i-1]; // This is a division by the diagonal of Beta, but the diagonal of Beta is inverted, so we multiply instead by the diagonal inverted

    }

    for (unsigned int k = NN_+1 ; k > 0 ; k--){

        for(unsigned int i = nn_ ; i > 0 ; i--){

            for(unsigned int p = i+1 ; p <= nn_ ; p++){

                d[(k-1)*nn_+i-1] -= Beta[k-1][i-1][p-1] * d[(k-1)*nn_+(p-1)];

            }
                
            for(unsigned int p = 0 ; p < nn_ ; p++){
            
                d[(k-1)*nn_+i-1] -= Alpha[k-1][i-1][p] * d[(k)*nn_+p];
            
            }

            d[(k-1)*nn_+i-1] *= Beta[k-1][i-1][i-1]; // This is a division by the diagonal of Beta, but the diagonal of Beta is inverted, so we multiply instead by the diagonal inverted

        }

    }
    

}

#ifdef SCALAR_RHO
void solve_banded_QRST_sys(const double (*Q_rho_i)[nn_], const double (*R_rho_i)[mm_], const double (*S_rho_i)[mm_], const double (*T_rho_i)[nn_], double *z, double *d){

    for (unsigned int i = 0 ; i < NN_ ; i++){ // Moving in groups of nn+mm components

        for (unsigned int j = 0 ; j < nn_ ; j++){ // Moving inside the groups
            
            for (unsigned int k = 0 ; k < nn_ ; k++){ // Multiplying the rows by the corresponding part of the independent term vector
                
                z[i*nm_+j] += Q_rho_i[j][k] * d[i*nm_+k];

            }

        }

        for (unsigned int j = nn_ ; j < nm_ ; j++){

            for (unsigned int k = 0 ; k < mm_ ; k++){

                z[i*nm_+j] += R_rho_i[j-nn_][k] * d[i*nm_+nn_+k];

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
#else
void solve_banded_QRST_sys(const double (*Q_rho_i)[nn_][nn_], const double (*R_rho_i)[mm_][mm_], const double (*S_rho_i)[mm_], const double (*T_rho_i)[nn_], double *z, double *d){

    for (unsigned int i = 0 ; i < NN_ ; i++){ // Moving in groups of nn+mm components

        for (unsigned int j = 0 ; j < nn_ ; j++){ // Moving inside the groups
            
            for (unsigned int k = 0 ; k < nn_ ; k++){ // Multiplying the rows by the corresponding part of the independent term vector
                
                z[i*nm_+j] += Q_rho_i[i][j][k] * d[i*nm_+k];

            }

        }

        for (unsigned int j = nn_ ; j < nm_ ; j++){

            for (unsigned int k = 0 ; k < mm_ ; k++){

                z[i*nm_+j] += R_rho_i[i][j-nn_][k] * d[i*nm_+nn_+k];

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

#endif

#if MEASURE_TIME == 1

spcies_snippet_get_elapsed_time();

spcies_snippet_read_time();

#endif

