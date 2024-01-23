// proj_SOC3() - Returns, in x, the projection of x onto the shifted-SOC given by alpha and d
// The function assumes that the dimension of x is 3, which is the case in the HMPC solver

void proj_SOC3(double *x, double alpha, double d){

    double x_0 = x[0];
    double x_norm = 0.0;
    double x_proj_step;
    double corrected_x_0;

    // Compute x_norm (the norm of the 2nd-to-last elements of x)
    for(unsigned int j = 1; j < 3; j++){
        x_norm += x[j]*x[j];
    }
    x_norm = sqrt(x_norm);

    // Compute auxliary term
    corrected_x_0 = alpha*(x_0 - d);

    // Project onto the shifted-SOC
    if(x_norm <= corrected_x_0){
    } else if(x_norm <= -corrected_x_0){
        x[0] = d; 
        for(unsigned int j = 1; j < 3; j++){
            x[j] = 0.0;
        }
    } else{
        x_proj_step = (corrected_x_0 + x_norm)/(2*x_norm);
        x[0] = x_proj_step*x_norm*alpha + d;
        for(unsigned int j = 1; j < 3; j++){
            x[j] = x_proj_step*x[j];
        }
    }

}
