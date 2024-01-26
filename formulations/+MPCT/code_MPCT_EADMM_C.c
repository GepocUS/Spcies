/**
 * Sparse EADMM solver for the MPCT formulation
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

void MPCT_EADMM(double *x0_in, double *xr_in, double *ur_in, double *u_opt, int *k_in, int *e_flag, sol_$INSERT_NAME$ *sol){

#if MEASURE_TIME == 1

#if WIN32
static LARGE_INTEGER start, post_update, post_solve, post_polish;
#else // If Linux
struct timespec start, post_update, post_solve, post_polish;
#endif

read_time(&start);

#endif

// Initialize solver variables
int done = 0; // Flag used to determine when the algorithm should exit
int k = 0; // Number of iterations
double x0[nm_] = {0.0}; // Current system state
double xr[nn_] = {0.0}; // State reference
double ur[mm_] = {0.0}; // Control input reference
double z1[NN_+1][nm_] = {{0.0}}; // Decision variable z1
double z2[nm_] = {0.0}; // Decision variable z2
double z3[NN_+1][nm_] = {{0.0}}; // Decision variable z3
#ifndef IS_DIAG
double z3_aux[NN_+1][nm_] = {{0.0}}; // Auxiliary vector for computing z3 when Q and R are non-diagonal
#endif
double z2_prev[nm_] = {0.0}; // Value of z2 in the previous iteration
double z3_prev[NN_+1][nm_] = {{0.0}}; // Value of z3 in the previous iteration
double lambda[NN_+3][nm_] = {{0.0}}; // Dual variables
double q2[nm_] = {0.0}; // Vector used to solve the z2 QP
double mu[NN_][nn_] = {{0.0}}; // Used to solve the system of equations
double res[NN_+3][nm_] = {{0.0}}; // Vector to store the residual
double res_abs = 0.0; // For storing the absolute value of res
double res_z2; // Variable used to determine if z2 is converging to a fixed point
double res_z3; // Variable used to determine if z2 is converging to a fixed point
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

// Measure time
#if MEASURE_TIME == 1
read_time(&post_update);
get_elapsed_time(&sol->update_time, &post_update, &start);
#endif

// Algorithm
while(done == 0){

    k += 1; // Increment iteration counter

    // Save the past value of z2 and z3
    memcpy(z2_prev, z2, sizeof(double)*nm_);
    memcpy(z3_prev, z3, sizeof(double)*(NN_+1)*nm_);

    //********** Problem P1 **********//
    // This problem updates z1

    // Compute the first nm_ elements of z1
    for(unsigned int j = 0; j < nm_; j++){
        z1[0][j] = (rho[0][j]*( z3[0][j] + z2[j]) + rho_0[j]*x0[j] + lambda[1][j] - lambda[0][j])*H1i[0][j];
        z1[0][j] = (z1[0][j] > LB_0[j]) ? z1[0][j] : LB_0[j]; // Saturate to lower bound
        z1[0][j] = (z1[0][j] > UB_0[j]) ? UB_0[j] : z1[0][j]; // Saturate to upper bound
    }

    // Compute all the other elements of z1 except for the last nm_
    for(unsigned int l = 1; l < NN_; l++){
        for(unsigned int j = 0; j < nm_; j++){
            z1[l][j] = (rho[l][j]*( z3[l][j] + z2[j]) + lambda[l+1][j])*H1i[l][j];
            z1[l][j] = (z1[l][j] > LB[j]) ? z1[l][j] : LB[j]; // Saturate to lower bound
            z1[l][j] = (z1[l][j] > UB[j]) ? UB[j] : z1[l][j]; // Saturate to upper bound
        }
    }

    // Compute the last nm_ elements of z1
    for(unsigned int j = 0; j < nm_; j++){
        z1[NN_][j] = (rho[NN_][j]*z3[NN_][j] + (rho[NN_][j] + rho_s[j])*z2[j] + lambda[NN_+1][j] + lambda[NN_+2][j])*H1i[NN_][j];
        z1[NN_][j] = (z1[NN_][j] > LB_s[j]) ? z1[NN_][j] : LB_s[j]; // Saturate to lower bound
        z1[NN_][j] = (z1[NN_][j] > UB_s[j]) ? UB_s[j] : z1[NN_][j]; // Saturate to upper bound
    }

    //********** Problem P2 **********//
    // This problem updates z2

    // Compute q2
    for(unsigned int j = 0; j < nm_; j++){
        q2[j] = rho[NN_][j]*z3[NN_][j] - (rho[NN_][j] + rho_s[j])*z1[NN_][j] + lambda[NN_+1][j] + lambda[NN_+2][j];
    }
    for(unsigned int j = 0; j < nn_; j++){
        for(unsigned int i = 0; i < nn_; i++){
            q2[j] = q2[j] + T[j][i]*xr[i];
        }
    }
    for(unsigned int j = 0; j < mm_; j++){
        for(unsigned int i = 0; i < mm_; i++){
            q2[j+nn_] = q2[j+nn_] + S[j][i]*ur[i];
        }
    }
    for(unsigned int l = 0; l < NN_; l++){
        for(unsigned int j = 0; j < nm_; j++){
            q2[j] = q2[j] + rho[l][j]*(z3[l][j] - z1[l][j]) + lambda[l+1][j];
        }
    }

    // Compute z2

    for(unsigned int j = 0; j < nm_; j++){
        z2[j] = 0;
        for(unsigned int i = 0; i < nm_; i++){
           z2[j] = z2[j] + W2[j][i]*q2[i];
        }
    }

    //********** Problem P3 **********//
    // This problem updates z3
     
    // Compute q3. I store it in z3 to save some memory and computations further ahead

    // Compute the first nm_ elements of q3
    for(unsigned int j = 0; j < nm_; j++){
        z3[0][j] = rho[0][j]*(z2[j] - z1[0][j]) + lambda[1][j];
    }

    // Compute all the other elements of q3 except for the last nm_
    for(unsigned int l = 1; l < NN_; l++){
       for(unsigned int j = 0; j < nm_; j++){
          z3[l][j] = rho[l][j]*(z2[j] - z1[l][j]) + lambda[l+1][j];
       }
    }

    // Compute the last nm_ elements of q3
    for(unsigned int j = 0; j < nm_; j++){
        z3[NN_][j] = rho[NN_][j]*(z2[j] - z1[NN_][j]) + lambda[NN_+1][j];
    }

    // Compute r.h.s of the Wc system of equations, i.e., -G3*H3^(-1)*q3
    // I store it in mu to save a bit of memory
    #if IS_DIAG == 1
    for(unsigned int l = 0; l < NN_; l++){
        for(unsigned int j = 0; j < nn_; j++){
            mu[l][j] = H3i[l+1][j]*z3[l+1][j];
            for(unsigned int i = 0; i < nm_; i++){
                mu[l][j] = mu[l][j] - AB[j][i]*H3i[l][i]*z3[l][i];
            }
        }
    }
    #else

    // -eye(nn_) terms except the last one
    for(unsigned int l = 0; l < NN_-1; l++){
        for(unsigned int j = 0; j < nn_; j++){
            mu[l][j] = 0.0;
            for(unsigned int i = 0; i < nn_; i++){
                mu[l][j] += Q_bi[j][i]*z3[l+1][i];
            }
        }
    }
    // Last -eye(nn_) term
    for(unsigned int j = 0; j < nn_; j++){
        mu[NN_-1][j] = 0.0;
        for(unsigned int i = 0; i < nn_; i++){
            mu[NN_-1][j] += Q_mi[j][i]*z3[NN_][i];
        }
    }
    // AB terms for the first nn_ elements
    for(unsigned int j = 0; j < nn_; j++){
        for(unsigned int i = 0; i < nm_; i++){
            mu[0][j] -= AB_mi[j][i]*z3[0][i];
        }
    }
    // AB terms for all the other elements
    for(unsigned int l = 1; l < NN_; l++){
        for(unsigned int j = 0; j < nn_; j++){
            for(unsigned int i = 0; i < nm_; i++){
                mu[l][j] -= AB_bi[j][i]*z3[l][i];
            }
        }
    }

    #endif

    // Compute mu, the solution of the system of equations W*mu = -G3'*H3^(-1)*q3

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

    // Compute z3 (note that, from before, we have that at this point z3 = q3)

    // Compute the first nm_ elements of z3
    for(unsigned int j = 0; j < nm_; j++){
        for(unsigned int i = 0; i <  nn_; i++){
            z3[0][j] = z3[0][j] + AB[i][j]*mu[0][i];
        }
    }

    // Compute all the other elements of z3 except for the last nm_
    for(unsigned int l = 1; l < NN_; l++){
        for(unsigned int j = 0; j < nn_; j++){
            z3[l][j] = z3[l][j] - mu[l-1][j];
        }
        for(unsigned int j = 0; j < nm_; j++){
            for(unsigned int i = 0; i < nn_; i++){
               z3[l][j] = z3[l][j] + AB[i][j]*mu[l][i];
            }
        }
    }

    // Compute the last nm_ elements of z3
    for(unsigned int j = 0; j < nn_; j++){
        z3[NN_][j] =  z3[NN_][j] - mu[NN_-1][j];
    }

    #if IS_DIAG == 1
    for(unsigned int l = 0; l < NN_+1; l++){
        for(unsigned int j = 0; j < nm_; j++){
            z3[l][j] = -H3i[l][j]*z3[l][j];
        }
    }
    #else

    // Assign z3 to z3_aux
    memcpy(z3_aux, z3, sizeof(double)*(NN_+1)*nm_);
    // Assign zeros to z3
    memset(z3, 0.0, sizeof(double)*(NN_+1)*nm_);

    // Compute the first nm_ elements
    for(unsigned int j = 0; j < nn_; j++){
        for(unsigned int i = 0; i < nn_; i++){
            z3[0][j] -= Q_mi[j][i]*z3_aux[0][i];
        }
    }
    for(unsigned int j = 0; j < mm_; j++){
        for(unsigned int i = 0; i < mm_; i++){
            z3[0][j+nn_] -= R_bi[j][i]*z3_aux[0][i+nn_];
        }
    }

    // Compute all other elements except the last nm_
    for(unsigned int l = 1; l < NN_; l++){
        for(unsigned int j = 0; j < nn_; j++){
            for(unsigned int i = 0; i < nn_; i++){
                z3[l][j] -= Q_bi[j][i]*z3_aux[l][i];
            }
        }
        for(unsigned int j = 0; j < mm_; j++){
            for(unsigned int i = 0; i < mm_; i++){
                z3[l][j+nn_] -= R_bi[j][i]*z3_aux[l][i+nn_];
            }
        }
    }

    // Compute the last nm_ elements
    for(unsigned int j = 0; j < nn_; j++){
        for(unsigned int i = 0; i < nn_; i++){
            z3[NN_][j] -= Q_mi[j][i]*z3_aux[NN_][i];
        }
    }
    for(unsigned int j = 0; j < mm_; j++){
        for(unsigned int i = 0; i < mm_; i++){
            z3[NN_][j+nn_] -= R_mi[j][i]*z3_aux[NN_][i+nn_];
        }
    }

    #endif

    //********** Residual and lambda **********//

    // Compute the first nn_ elements of the residual
    for(unsigned int j = 0; j < nn_; j++){
        res[0][j] = z1[0][j] - x0[j];
    }

    // Compute all the other elements except of res for the last nm_
    for(unsigned int l = 0; l < NN_+1; l++){
        for(unsigned int j = 0; j < nm_; j++){
            res[l+1][j] = z2[j] + z3[l][j] - z1[l][j];
        }
    }

    // Compute the last nm_ elements of res
    for(unsigned int j = 0; j < nm_; j++){
        res[NN_+2][j] = z2[j] - z1[NN_][j];
    }

    // Compute the first nn_ elements of lambda
    for(unsigned int j = 0; j < nn_; j++){
        lambda[0][j] = lambda[0][j] + rho_0[j]*res[0][j];
    }

    // Compute all the other elements of lambda except for the last nm_
    for(unsigned int l = 1; l < NN_+2; l++){
        for(unsigned int j = 0; j < nm_; j++){
            lambda[l][j] = lambda[l][j] + rho[l-1][j]*res[l][j];
        }
    }

    // Compute the last nm_ elements of lambda
    for(unsigned int j = 0; j < nm_; j++){
        lambda[NN_+2][j] = lambda[NN_+2][j] + rho_s[j]*res[NN_+2][j];
    }

    //********** Exit condition **********//

    // Check if the exit condition is satisfied

    res_flag = 0; // Reset the residual flag

    for(unsigned int j = 0; j < nm_; j++){
        res_z2 = z2_prev[j] - z2[j];
        res_z2 = (res_z2 > 0.0) ? res_z2 : -res_z2;
        if(res_z2 > tol){
            res_flag = 1;
            break;
        }
    }

    if(res_flag == 0){
        for(unsigned int l = 0; l < NN_+3; l++){
            for(unsigned int j = 0; j < nm_; j++){
                res_abs = (res[l][j] > 0.0) ? res[l][j] : -res[l][j]; // Get absolute value
                if(res_abs > tol){
                    res_flag = 1;
                    break;
                }
            }
            if(res_flag == 1){
                break;
            }
        }
    }

    if(res_flag == 0){
        for(unsigned int l = 0; l < NN_+1; l++){
            for(unsigned int j = 0; j < nm_; j++){
                res_z3 = z3_prev[l][j] - z3[l][j];
                res_z3 = (res_z3 > 0.0) ? res_z3 : -res_z3;
                if(res_z3 > tol){
                    res_flag = 1;
                    break;
                }
            }
            if(res_flag == 1){
                break;
            }
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

}

// Measure time
#if MEASURE_TIME == 1
read_time(&post_solve);
get_elapsed_time(&sol->solve_time, &post_solve, &post_update);
#endif

//********** Final steps **********//

// Extract control action
#if in_engineering == 1
for(unsigned int j = 0; j < mm_; j++){
    u_opt[j] = z1[0][nn_+j]*scaling_i_u[j] + OpPoint_u[j];
}
#endif
#if in_engineering == 0
for(unsigned int j = 0; j < mm_; j++){
    u_opt[j] = z1[0][nn_+j];
}
#endif

// Return number of iterations
*k_in = k; // Number of iterations

// Return optimal variables (in vector form)
#ifdef DEBUG

// Primal variables
int count = -1;
for(unsigned int l = 0; l < NN_+1; l++){
    for(unsigned int j = 0; j < nm_; j++){
        count++;
        sol->z1[count] = z1[l][j];
        sol->z3[count] = z3[l][j];
    }
}

count = -1;
for(unsigned int j = 0; j < nm_; j++){
    count++;
    sol->z2[count] = z2[j];
}

// Dual variables
count = -1;
for(unsigned int j = 0; j < nn_; j++){
    count++;
    sol->lambda[count] = lambda[0][j];
}
for(unsigned int l = 1; l < NN_+3; l ++){
    for(unsigned int j = 0; j < nn_; j++){
        count++;
        sol->lambda[count] = lambda[l][j];
    }
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

