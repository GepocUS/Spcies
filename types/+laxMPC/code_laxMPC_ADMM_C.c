/**
 * Sparse ADMM solver for the lax MPC formulation
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

#ifdef CONF_MATLAB

#if time_varying == 0
void laxMPC_ADMM(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *u_opt, double *pointer_k, double *e_flag, double *z_opt, double *v_opt, double *lambda_opt){
#else
// void laxMPC_ADMM(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *pointer_A, double *pointer_B, double *pointer_Q, double *pointer_R, double *pointer_T, double *u_opt, double *pointer_k, double *e_flag, double *z_opt, double *v_opt, double *lambda_opt){
void laxMPC_ADMM(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *pointer_A, double *pointer_B, double *pointer_Q, double *pointer_R, double *u_opt, double *pointer_k, double *e_flag, double *z_opt, double *v_opt, double *lambda_opt){
#endif

#else
    
#if time_varying == 0
void laxMPC_ADMM(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *u_opt, int *pointer_k, int *e_flag, solution *sol){
#else
// void laxMPC_ADMM(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *pointer_A, double *pointer_B, double *pointer_Q, double *pointer_R, double *pointer_T, double *u_opt, int *pointer_k, int *e_flag, solution *sol){
void laxMPC_ADMM(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *pointer_A, double *pointer_B, double *pointer_Q, double *pointer_R, double *u_opt, int *pointer_k, int *e_flag, solution *sol){
#endif

#endif

    // Initialize ADMM variables
    int done = 0; // Flag used to determine when the algorithm should exit
    #ifdef CONF_MATLAB
    double k = 0.0; // Number of iterations. In the Matlab case it is easier if it is defined as a double
    #else
    int k = 0; // Number of iterations
    #endif
    double x0[nn]; // Current system state
    double xr[nn]; // State reference
    double ur[mm]; // Control input reference
    #if time_varying == 1
        double A[nn][nn];
        double B[nn][mm];
        double AB[nn][nm];
        double Q[nn];
        double R[mm];
//         double T[nn][nn];
        double Hi[NN-1][nm]; // Falta calcularles en línea el valor a estos tres
        double Hi_0[mm];
        double Hi_N[nn][nn];
        double R_rho_i[mm] = {0.0}; // 1./(R+rho*eye(mm))
        double Q_rho_i[nn] = {0.0}; // 1./(Q+rho*eye(nn))
        double Alpha[NN-1][nn][nn] = {{{0.0}}};
        double Beta[NN][nn][nn] = {{{0.0}}};
        double inv_Beta[nn][nn] = {{0.0}}; // Inverse of only the current beta is stored
    #endif
    double v[NN-1][nm] = {{0.0}}; // Decision variables v
    double v_0[mm] = {0.0};
    double v_N[nn] = {0.0};
    double lambda[NN-1][nm] = {{0.0}}; // Dual variables lambda
    double lambda_0[mm] = {0.0};
    double lambda_N[nn] = {0.0};
    double z[NN-1][nm] = {{0.0}}; // Decision variables z
    double z_0[mm] = {0.0};
    double z_N[nn] = {0.0};
    double v1[NN-1][nm] = {{0.0}}; // Value of the decision variables z at the last iteration
    double v1_0[mm] = {0.0};
    double v1_N[nn] = {0.0};
    double aux_N[nn] = {0.0}; // Auxiliary array used for multiple purposes
    double mu[NN][nn] = {{0.0}}; // Used to solve the system of equations
    unsigned int res_flag = 0; // Flag used to determine if the exit condition is satisfied
    double res_fixed_point; // Variable used to determine if a fixed point has been reached
    double res_primal_feas; // Variable used to determine if primal feasibility is satisfied
    double b[nn] = {0.0}; // First nn components of vector b (the rest are known to be zero)
    double q[nm] = {0.0};
    double qT[nn] = {0.0};
    
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

    #if time_varying == 1
    for(unsigned int i = 0; i < nn; i++){
        Q[i] = pointer_Q[i];
        Q_rho_i[i] = 1/(Q[i]+rho);
        for(unsigned int j = 0; j < nn; j++){
            A[i][j] = pointer_A[i+j*nn];
//             T[i][j] = pointer_T[i+j*nn];
            Hi_N[i][j] = T_rho_i[i][j];
            // Constructing AB: Part of A
            AB[i][j] = A[i][j];
        }
        for(unsigned int j=0; j < mm; j++){
            if (i==0){
                R[j] = pointer_R[j];
                R_rho_i[j] = 1/(R[j]+rho);
                Hi_0[j] = R_rho_i[j];
            }
            B[i][j] = pointer_B[i+j*nn];
            // Constructing AB: Part of B
            AB[i][nn+j] = B[i][j];
        }

    }

    // Constructing Hi
    for(unsigned int i=0 ; i<NN-1 ; i++){
        for(unsigned int j=0 ; j<nm ; j++){
            if(j<nn){
                Hi[i][j] = Q_rho_i[j];
            }
            else{
                Hi[i][j] = R_rho_i[j-nn];
            }
        }
    }
    #endif
    
    #if time_varying == 1
    // For the case of time_varying==true, now that all elements are caught,
    // alpha's and beta's can be obtained from explicit formulas
    for(unsigned int h = 0; h < NN ; h++){
        for(unsigned int i = 0 ; i < nn ; i++){
            for(unsigned int j = 0 ; j < nn ; j++){
                if (h==0){ //Beta{0}
                    if (i==j){

                        for (unsigned int m = 0 ; m < mm ; m++){
                            Beta[h][i][j] += B[i][m]*R_rho_i[m]*B[j][m];
                        }
                        
                        Beta[h][i][j] += Q_rho_i[i];

                        if(i>0){
                            for(unsigned int l = 0 ; l <= i-1 ; l++){
                                Beta[h][i][j] -= Beta[h][l][i]*Beta[h][l][i];
                            }
                            
                        }

                        Beta[h][i][j] = sqrt(Beta[h][i][j]);

                    }

                    else if (j>i){

                        for (unsigned int m = 0 ; m<mm ; m++){
                            Beta[h][i][j] += B[i][m]*R_rho_i[m]*B[j][m];
                        }

                        if(i>0){
                            for(unsigned int l = 0 ; l <= i-1 ; l++){
                                Beta[h][i][j] -= Beta[h][l][i]*Beta[h][l][j];
                            }
                            
                        }

                        Beta[h][i][j] = Beta[h][i][j]/Beta[h][i][i];

                    }
                    
                }

                else if (h<NN-1){ //Beta{1} to Beta{N-1}
                    if(i==j){
                        for(unsigned int n = 0 ; n < nn ; n++){
                            Beta[h][i][j] += A[i][n]*Q_rho_i[n]*A[j][n];
                        }

                        for(unsigned int m = 0 ; m < mm ; m++){
                            Beta[h][i][j] += B[i][m]*R_rho_i[m]*B[j][m];
                        }

                        Beta[h][i][j] += Q_rho_i[i];

                        for(unsigned int k = 0 ; k < nn ; k++){
                            Beta[h][i][j] -= Alpha[h-1][k][i]*Alpha[h-1][k][j];
                        }
                        
                        if(i>0){
                            for(unsigned int l = 0 ; l<=i-1 ; l++){
                                Beta[h][i][j] -= Beta[h][l][i]*Beta[h][l][i];
                            }
                        }
                        
                        Beta[h][i][j] = sqrt(Beta[h][i][j]);

                    }

                    else if (j>i){
                        
                        for(unsigned int n = 0 ; n < nn ; n++){
                            Beta[h][i][j] += A[i][n]*Q_rho_i[n]*A[j][n];
                        }

                        for(unsigned int m = 0 ; m < mm ; m++){
                            Beta[h][i][j] += B[i][m]*R_rho_i[m]*B[j][m];
                        }

                        for(unsigned int k = 0 ; k < nn ; k++){
                            Beta[h][i][j] -= Alpha[h-1][k][i]*Alpha[h-1][k][j];
                        }

                        if(i>0){
                            for(unsigned int l = 0 ; l<=i-1 ; l++){
                                Beta[h][i][j] -= Beta[h][l][i]*Beta[h][l][j];
                            }
                        }

                        Beta[h][i][j] = Beta[h][i][j]/Beta[h][i][i];

                    }

                }

                else{ //Beta{N}

                    if(i==j){
                        for(unsigned int n=0 ; n<nn ; n++){
                            Beta[h][i][j] += A[i][n] * Q_rho_i[n] * A[j][n];                         
                        }
                        for (unsigned int m=0 ; m<mm ; m++){
                            Beta[h][i][j] += B[i][m] * R_rho_i[m] * B[j][m];
                        }

                        Beta[h][i][j] += T_rho_i[i][j];

                        for(unsigned int k=0 ; k<nn ; k++){
                            Beta[h][i][j] -= Alpha[h-1][k][i]*Alpha[h-1][k][j];
                        }

                        if(i>0){
                            for(unsigned int l=0 ; l<=i-1 ; l++){
                                Beta[h][i][j] -= Beta[h][l][i] * Beta[h][l][i];
                            }
                        }

                        Beta[h][i][j] = sqrt(Beta[h][i][j]);

                    }

                    else if(j>i){
                        for(unsigned int n=0 ; n<nn ; n++){
                            Beta[h][i][j] += A[i][n] * Q_rho_i[n] * A[j][n];
                        }
                        for(unsigned int m=0 ; m<mm ; m++){
                            Beta[h][i][j] += B[i][m] * R_rho_i[m] * B[j][m];
                        }

                        Beta[h][i][j] += T_rho_i[i][j];

                        for(unsigned int k=0 ; k<nn ; k++){
                            Beta[h][i][j] -= Alpha[h-1][k][i] * Alpha[h-1][k][j];
                        }

                        if(i>0){
                            for(unsigned int l=0 ; l<=i-1 ; l++){
                                Beta[h][i][j] -= Beta[h][l][i] * Beta[h][l][j];
                            }
                        }

                        Beta[h][i][j] = Beta[h][i][j]/Beta[h][i][i];

                    }
                    
                }
                  

            }
        }

        // Calculation of Alpha's
        if (h < NN-1){
            // Calculation of the inverse of the current Beta, needed for current Alpha
//             memset(inv_Beta, 0, sizeof(inv_Beta)); // Reset of inv_Beta when a new Beta is calculated
            
            for(unsigned int i=0 ; i<nn ; i++){
                for(unsigned int j=0 ; j<nn ; j++){
                    inv_Beta[i][j] = 0;
                }
            }


            for (int i=nn-1 ; i>=0 ; i--){
                for (unsigned int j=0 ; j<nn ; j++){
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

            for (unsigned int i=0 ; i<nn ; i++){
                for (unsigned int j=0 ; j<nn ; j++){
                    for (unsigned int k=0 ; k<=i ; k++){
                        Alpha[h][i][j] -= inv_Beta[k][i] * A[j][k] * Q_rho_i[k];
                    }
                }
            }

    
        }

    }

    for (unsigned int h=0 ; h<NN ; h++){

        for (unsigned int i=0 ; i<nn ; i++){
            Beta[h][i][i] = 1/Beta[h][i][i]; // We need to make the component-wise inversion of the diagonal elements of Beta's
        }
        
    }

    // end of calculation of alpha's and beta's

    // Inverting Q and R, needed for the rest of the program
    for(unsigned int i=0 ; i<nn ; i++){
        Q[i] = -Q[i];
    }
    for(unsigned int i=0 ; i<mm ; i++){
        R[i] = -R[i];
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
        qT[j] = 0.0;
        for(unsigned int i = 0; i < nn; i++){
            qT[j] = qT[j] + T[j][i]*xr[i];
        }
    }
    for(unsigned int j = 0; j < mm; j++){
        q[j+nn] = R[j]*ur[j];
    }

    // Algorithm
    while(done == 0){

        k += 1; // Increment iteration counter

        // Step 0: Save the value of v into variable v1
        memcpy(v1_0, v_0, sizeof(double)*mm);
        memcpy(v1, v, sizeof(double)*(NN-1)*nm);
        memcpy(v1_N, v_N, sizeof(double)*nn);

        // Step 1: Minimize w.r.t. z

        // Compute vector q_hat = lambda - rho*v
        // I store vector q_hat in z because I save memory and computation
        
        // Compute the first mm elements
        for(unsigned int j = 0; j < mm; j++){
            #ifdef SCALAR_RHO
            z_0[j] = q[j+nn] + lambda_0[j] - rho*v_0[j];
            #else
            z_0[j] = q[j+nn] + lambda_0[j] - rho_0[j]*v_0[j];
            #endif
        }

        // Compute all the other elements except for the last nn
        for(unsigned int l = 0; l < NN-1; l++){
            for(unsigned int j = 0; j < nm; j++){
                #ifdef SCALAR_RHO
                z[l][j] = q[j] + lambda[l][j] - rho*v[l][j];
                #else
                z[l][j] = q[j] + lambda[l][j] - rho[l][j]*v[l][j];
                #endif
            }
        }

        // Compute the last nn elements
        for(unsigned int j = 0; j < nn; j++){
            #ifdef SCALAR_RHO
            z_N[j] = qT[j] + lambda_N[j] - rho*v_N[j];
            #else
            z_N[j] = qT[j] + lambda_N[j] - rho_N[j]*v_N[j];
            #endif
        }

        // Compute r.h.s of the Wc system of equations, i.e., -G'*H_hat^(-1)*q_hat - b
        // I store it in mu to save a bit of memory

        // Compute the first nn elements
        for(unsigned int j = 0; j < nn; j++){
            mu[0][j] = Hi[0][j]*z[0][j] - b[j];
            for(unsigned int i = 0; i < mm; i++){
                mu[0][j] = mu[0][j] - AB[j][i+nn]*Hi_0[i]*z_0[i];
            }
        }

        // Compute all the other elements except for the last nn
        for(unsigned int l = 1; l < NN-1; l++){
            for(unsigned int j = 0; j < nn; j++){
                mu[l][j] = Hi[l][j]*z[l][j];
                for(unsigned int i = 0; i < nm; i++){
                    mu[l][j] = mu[l][j] - AB[j][i]*Hi[l-1][i]*z[l-1][i];
                }
            }
        }

        // Compute the last nn elements
        for(unsigned int j = 0; j < nn; j++){
            mu[NN-1][j] = 0.0;
            for(unsigned int i = 0; i < nn; i++){
                mu[NN-1][j] = mu[NN-1][j] + Hi_N[j][i]*z_N[i];
            }
            for(unsigned int i = 0; i < nm; i++){
                mu[NN-1][j] = mu[NN-1][j] - AB[j][i]*Hi[NN-2][i]*z[NN-2][i];
            }
        }
        
        // Compute mu, the solution of the system of equations W*mu = -G'*H^(-1)*q_hat - beq

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

        // Compute z (note that, from before, we have that at this point z = q_hat)

        // Compute the first mm elements
        for(unsigned int j = 0; j < mm; j++){
            for(unsigned int i = 0; i < nn; i++){
                z_0[j] = z_0[j] + AB[i][j+nn]*mu[0][i];
            }
            z_0[j] = -Hi_0[j]*z_0[j];
        }

        // Compute all the other elements except for the last nn
        for(unsigned int l = 0; l < NN-1; l++){
            for(unsigned int j = 0; j < nn; j++){
                z[l][j] = z[l][j] - mu[l][j];
            }
            for(unsigned int j = 0; j < nm; j++){
                for(unsigned int i = 0; i < nn; i++){
                    z[l][j] = z[l][j] + AB[i][j]*mu[l+1][i];
                }
                z[l][j] = -Hi[l][j]*z[l][j];
            }
        }

        // Compute the last nn elements
        for(unsigned int j = 0; j < nn; j++){
            aux_N[j] = z_N[j] - mu[NN-1][j];
        }
        for(unsigned int j = 0; j < nn; j++){
            z_N[j] = 0.0;
            for(unsigned int i = 0; i < nn; i++){
                z_N[j] = z_N[j] - Hi_N[j][i]*aux_N[i];
            }
        }

        // Step 2: Minimize w.r.t. v

        // Compute the first mm variables
        for(unsigned int j = 0; j < mm; j++){
            #ifdef SCALAR_RHO
            v_0[j] = z_0[j] + rho_i*lambda_0[j];
            #else
            v_0[j] = z_0[j] + rho_i_0[j]*lambda_0[j];
            #endif
            #ifdef VAR_BOUNDS
            v_0[j] = (v_0[j] > LB0[j]) ? v_0[j] : LB0[j]; // maximum between v and the lower bound
            v_0[j] = (v_0[j] > UB0[j]) ? UB0[j] : v_0[j]; // minimum between v and the upper bound
            #else
            v_0[j] = (v_0[j] > LB[j+nn]) ? v_0[j] : LB[j+nn]; // maximum between v and the lower bound
            v_0[j] = (v_0[j] > UB[j+nn]) ? UB[j+nn] : v_0[j]; // minimum between v and the upper bound
            #endif
        }

        // Compute all the other elements except the last nn
        for(unsigned int l = 0; l < NN-1; l++){
            for(unsigned int j = 0; j < nm; j++){
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

        // Compute the last nn elements
        for(unsigned int j = 0; j < nn; j++){
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
        for(unsigned int j = 0; j < mm; j++){
            #ifdef SCALAR_RHO
            lambda_0[j] = lambda_0[j] + rho*( z_0[j] - v_0[j] );
            #else
            lambda_0[j] = lambda_0[j] + rho_0[j]*( z_0[j] - v_0[j] );
            #endif
        }

        // Compute all the other elements except for the last nn
        for(unsigned int l = 0; l < NN-1; l++){
            for(unsigned int j = 0; j < nm; j++){
                #ifdef SCALAR_RHO
                lambda[l][j] = lambda[l][j] + rho*( z[l][j] - v[l][j] );
                #else
                lambda[l][j] = lambda[l][j] + rho[l][j]*( z[l][j] - v[l][j] );
                #endif
            }
        }

        // Compute the last nn elements
        for(unsigned int j = 0; j < nn; j++){
            #ifdef SCALAR_RHO
            lambda_N[j] = lambda_N[j] + rho*( z_N[j] - v_N[j] );
            #else
            lambda_N[j] = lambda_N[j] + rho_N[j]*( z_N[j] - v_N[j] );
            #endif
        }

        // Step 4: Compute the residual

        res_flag = 0; // Reset the residual flag

        // Compute the first mm elements
        for(unsigned int j = 0; j < mm; j++){
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

        // Compute the last NN elements
        if(res_flag == 0){
            for(unsigned int j = 0; j < nn; j++){
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
            for(unsigned int l = 0; l < NN-1; l++){
                for(unsigned int j = 0; j < nm; j++){
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
        u_opt[j] = v_0[j]*scaling_i_u[j] + OpPoint_u[j];
    }
    #endif
    #if in_engineering == 0
    for(unsigned int j = 0; j < mm; j++){
        u_opt[j] = v_0[j];
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

    // First mm variables
    int count = -1;
    for(unsigned int j = 0; j < mm; j++){
        count++;
        z_opt[count] = z_0[j];
        v_opt[count] = v_0[j];
        lambda_opt[count] = lambda_0[j];
    }

    // All other elements except the last nn
    for(unsigned int l = 0; l < nn; l++){
        for(unsigned int j = 0; j < nn; j++){
            count++;
            z_opt[count] = z[l][j];
            v_opt[count] = v[l][j];
            lambda_opt[count] = lambda[l][j];
        }
    }

    // Last nn elements
    for(unsigned int j = 0; j < nn; j++){
        count++;
        z_opt[count] = z_N[j];
        v_opt[count] = v_N[j];
        lambda_opt[count] = lambda_N[j];
    }

    #else

    // First mm variables
    int count = -1;
    for(unsigned int j = 0; j < mm; j++){
        count++;
        sol->z[count] = z_0[j];
        sol->v[count] = v_0[j];
        sol->lambda[count] = lambda_0[j];
    }

    // All other elements except the last nn
    for(unsigned int l = 0; l < NN-1; l++){
        for(unsigned int j = 0; j < nm; j++){
            count++;
            sol->z[count] = z[l][j];
            sol->v[count] = v[l][j];
            sol->lambda[count] = lambda[l][j];
        }
    }

    // Last nn elements
    for(unsigned int j = 0; j < nn; j++){
        count++;
        sol->z[count] = z_N[j];
        sol->v[count] = v_N[j];
        sol->lambda[count] = lambda_N[j];
    }

    #endif

    #endif

}

