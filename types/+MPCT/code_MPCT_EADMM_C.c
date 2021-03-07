/**
 * Sparse EADMM solver for the MPCT formulation
 * The current system state is given in "pointer_x0".
 * The state reference is given in "pointer_xr".
 * The input reference is given in "pointer_ur".
 * The optimal control action is returned in "u_opt".
 * The number of iterations is returned in "pointer_k".
 * The exit flag is returned in "e_flag".
 *       1: Algorithm converged successfully.
 *      -1: Algorithm did not converge within the maximum number of iterations. Returns current iterate.
 * The optimal decision variables are returned in "sol" (only if "DEGUB" is #defined).
 */

#include <stdio.h>

#ifdef CONF_MATLAB

void MPCT_EADMM(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *u_opt, double *pointer_k, double *e_flag, double *z1_opt, double *z2_opt, double *z3_opt, double *lambda_opt){

#else

void MPCT_EADMM(double *pointer_x0, double *pointer_xr, double *pointer_ur, double *u_opt, int *pointer_k, int *e_flag, solution_MPCT *sol){

#endif

// Initialize solver variables
int done = 0; // Flag used to determine when the algorithm should exit
#ifdef CONF_MATLAB
double k = 0.0; // Number of iterations. In the Matlab case it is easier if it is defined as a double
#else
int k = 0; // Number of iterations
#endif
double x0[nm] = {0.0}; // Current system state
double xr[nn] = {0.0}; // State reference
double ur[mm] = {0.0}; // Control input reference
double z1[NN+1][nm] = {{0.0}}; // Decision variable z1
double z2[nm] = {0.0}; // Decision variable z2
double z3[NN+1][nm] = {{0.0}}; // Decision variable z3
double lambda[NN+3][nm] = {{0.0}}; // Dual variables
double q2[nm] = {0.0}; // Vector used to solve the z2 QP
double mu[NN][nn] = {{0.0}}; // Used to solve the system of equations
double res[NN+3][nm] = {{0.0}}; // Vector to store the residual
double res_abs = 0.0; // For storing the absolute value of res
unsigned int res_flag = 0; // Flag used to determine if the exit condition is satisfied

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


// Algorithm
while(done == 0){

        k += 1; // Increment iteration counter

    //********** Problem P1 **********//
    // This problem updates z1

    // Compute the first nm elements of z1
    for(unsigned int j = 0; j < nm; j++){
        z1[0][j] = (rho[0][j]*( z3[0][j] + z2[j]) - rho_0[j]*x0[j] + lambda[1][j] - lambda[0][j])*H1i[0][j];
        z1[0][j] = (z1[0][j] > LB_0[j]) ? z1[0][j] : LB_0[j]; // Saturate to lower bound
        z1[0][j] = (z1[0][j] > UB_0[j]) ? UB_0[j] : z1[0][j]; // Saturate to upper bound
    }

    // Compute all the other elements of z1 except for the last nm
    for(unsigned int l = 1; l < NN; l++){
        for(unsigned int j = 0; j < nm; j++){
            z1[l][j] = (rho[l][j]*( z3[l][j] + z2[j]) + lambda[l+1][j])*H1i[l][j];
            z1[l][j] = (z1[l][j] > LB[j]) ? z1[l][j] : LB[j]; // Saturate to lower bound
            z1[l][j] = (z1[l][j] > UB[j]) ? UB[j] : z1[l][j]; // Saturate to upper bound
        }
    }

    // Compute the last nm elements of z1
    for(unsigned int j = 0; j < nm; j++){
        z1[NN][j] = (rho[NN][j]*z3[NN][j] + (rho[NN][j] + rho_s[j])*z2[j] + lambda[NN+1][j] + lambda[NN+2][j])*H1i[NN][j];
        z1[NN][j] = (z1[NN][j] > LB_s[j]) ? z1[NN][j] : LB_s[j]; // Saturate to lower bound
        z1[NN][j] = (z1[NN][j] > UB_s[j]) ? UB_s[j] : z1[NN][j]; // Saturate to upper bound
    }

    //********** Problem P2 **********//
    // This problem updates z2

    // Compute q2
    for(unsigned int j = 0; j < nm; j++){
        q2[j] = rho[NN][j]*z3[NN][j] - (rho[NN][j] + rho_s[j])*z1[NN][j] + lambda[NN+1][j] + lambda[NN+2][j];
    }
    for(unsigned int j = 0; j < nn; j++){
        for(unsigned int i = 0; i < nn; i++){
            q2[j] = q2[j] + T[j][i]*xr[i];
        }
    }
    for(unsigned int j = 0; j < mm; j++){
        for(unsigned int i = 0; i < mm; i++){
            q2[j+nn] = q2[j+nn] + S[j][i]*ur[i];
        }
    }
    for(unsigned int l = 0; l < NN; l++){
        for(unsigned int j = 0; j < nm; j++){
            q2[j] = q2[j] + rho[l][j]*(z3[l][j] - z1[l][j]) + lambda[l+1][j];
        }
    }

    // Compute z2

    for(unsigned int j = 0; j < nm; j++){
        z2[j] = 0;
        for(unsigned int i = 0; i < nm; i++){
           z2[j] = z2[j] + W2[j][i]*q2[i];
        }
    }

    //********** Problem P3 **********//
    // This problem updates z3
     
    // Compute q3. I store it in z3 to save some memory and computations further ahead

    // Compute the first nm elements of q3
    for(unsigned int j = 0; j < nm; j++){
        z3[0][j] = rho[0][j]*(z2[j] - z1[0][j]) + lambda[1][j];
    }

    // Compute all the other elements of q3 except for the last nm
    for(unsigned int l = 1; l < NN; l++){
       for(unsigned int j = 0; j < nm; j++){
          z3[l][j] = rho[l][j]*(z2[j] - z1[l][j]) + lambda[l+1][j];
       }
    }

    // Compute the last nm elements of q3
    for(unsigned int j = 0; j < nm; j++){
        z3[NN][j] = rho[NN][j]*(z2[j] - z1[NN][j]) + lambda[NN+1][j];
    }

    // Compute r.h.s of the Wc system of equations, i.e., -G3'*H3^(-1)*q3
    // I store it in mu to save a bit of memory
    for(unsigned int l = 0; l < NN; l++){
        for(unsigned int j = 0; j < nn; j++){
            mu[l][j] = H3i[l+1][j]*z3[l+1][j];
            for(unsigned int i = 0; i < nm; i++){
                mu[l][j] = mu[l][j] - AB[j][i]*H3i[l][i]*z3[l][i];
            }
        }
    }

    // Compute mu, the solution of the system of equations W*mu = -G3'*H3^(-1)*q3

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

    // Compute z3 (note that, from before, we have that at this point z3 = q3)

    // Compute the first nm elements of z3
    for(unsigned int j = 0; j < nm; j++){
        for(unsigned int i = 0; i <  nn; i++){
            z3[0][j] = z3[0][j] + AB[i][j]*mu[0][i];
        }
        z3[0][j] = -H3i[0][j]*z3[0][j];
    }

    // Compute all the other elements of z3 except for the last nm
    for(unsigned int l = 1; l < NN; l++){
        for(unsigned int j = 0; j < nn; j++){
            z3[l][j] = z3[l][j] - mu[l-1][j];
        }
        for(unsigned int j = 0; j < nm; j++){
            for(unsigned int i = 0; i < nn; i++){
               z3[l][j] = z3[l][j] + AB[i][j]*mu[l][i];
            }
           z3[l][j] = -H3i[l][j]*z3[l][j];
        }
    }

    // Compute the last nm elements of z3
    for(unsigned int j = 0; j < nn; j++){
        z3[NN][j] = H3i[NN][j]*(mu[NN-1][j] - z3[NN][j]);
    }
    for(unsigned int j = nn; j < nm; j++){
        z3[NN][j] = -H3i[NN][j]*z3[NN][j];
    }

    //********** Residual and lambda **********//

    // Compute the first nn elements of the residual
    for(unsigned int j = 0; j < nn; j++){
        res[0][j] = z1[0][j] - x0[j];
    }

    // Compute all the other elements except of res for the last nm
    for(unsigned int l = 0; l < NN+1; l++){
        for(unsigned int j = 0; j < nm; j++){
            res[l+1][j] = z2[j] + z3[l][j] - z1[l][j];
        }
    }

    // Compute the last nm elements of res
    for(unsigned int j = 0; j < nm; j++){
        res[NN+2][j] = z2[j] - z1[NN][j];
    }

    // Compute the first nn elements of lambda
    for(unsigned int j = 0; j < nn; j++){
        lambda[0][j] = lambda[0][j] + rho_0[j]*res[0][j];
    }

    // Compute all the other elements of lambda except for the last nm
    for(unsigned int l = 1; l < NN+2; l++){
        for(unsigned int j = 0; j < nm; j++){
            lambda[l][j] = lambda[l][j] + rho[l-1][j]*res[l][j];
        }
    }

    // Compute the last nm elements of lambda
    for(unsigned int j = 0; j < nm; j++){
        lambda[NN+2][j] = lambda[NN+2][j] + rho_s[j]*res[NN+2][j];
    }

    //********** Exit condition **********//

    // Check if the exit condition is satisfied

    res_flag = 0; // Reset the residual flag

    for(unsigned int l = 0; l < NN+3; l++){
        for(unsigned int j = 0; j < nm; j++){
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

//********** Final steps **********//

// Extract control action
#if in_engineering == 1
for(unsigned int j = 0; j < mm; j++){
    u_opt[j] = z1[0][nn+j]*scaling_i_u[j] + OpPoint_u[j];
}
#endif
#if in_engineering == 0
for(unsigned int j = 0; j < mm; j++){
    u_opt[j] = z1[0][nn+j];
}
#endif

// Return number of iterations
#ifdef CONF_MATLAB
pointer_k[0] = k; // Number of iterations
#else
*pointer_k = k; // Number of iterations
#endif

// Return optimal variables (in vector form)
#ifdef DEBUG

#ifdef CONF_MATLAB

// Primal variables
int count = -1;
for(unsigned int l = 0; l < NN+1; l++){
    for(unsigned int j = 0; j < nm; j++){
        count++;
        z1_opt[count] = z1[l][j];
        z3_opt[count] = z3[l][j];
    }
}

count = -1;
for(unsigned int j = 0; j < nm; j++){
    count++;
    z2_opt[count] = z2[j];
}

// Dual variables
count = -1;
for(unsigned int j = 0; j < nn; j++){
    count++;
    lambda_opt[count] = lambda[0][j];
}
for(unsigned int l = 1; l < NN+3; l ++){
    for(unsigned int j = 0; j < nm; j++){
        count++;
        lambda_opt[count] = lambda[l][j];
    }
}

#else

// Primal variables
int count = -1;
for(unsigned int l = 0; l < NN+1; l++){
    for(unsigned int j = 0; j < nm; j++){
        count++;
        sol->z1[count] = z1[l][j];
        sol->z3[count] = z3[l][j];
    }
}

count = -1;
for(unsigned int j = 0; j < nm; j++){
    count++;
    sol->z2[count] = z2[j];
}

// Dual variables
count = -1;
for(unsigned int j = 0; j < nn; j++){
    count++;
    sol->lambda[count] = lambda[0][j];
}
for(unsigned int l = 1; l < NN+3; l ++){
    for(unsigned int j = 0; j < nn; j++){
        count++;
        sol->lambda[count] = lambda[l][j];
    }
}

#endif

#endif

}

