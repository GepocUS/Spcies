%% def_options_MPCT_EADMM
% 
%  Returns the default options for the ADMM-based solver for the lax MPC formulation
% 
% Information about this formulation and the solver can be found at:
% 
% P. Krupa, D. Limon, T. Alamo, "Implementation of model predictive control in
% programmable logic controllers", Transactions on Control Systems Technology, 2020.
%
% OUTPUT:
%   - def_options: Structure containing the default options of the solver

function def_options = def_options_laxMPC_ADMM()

    def_options.rho = 1e-2;
    def_options.tol = 1e-4;
    def_options.k_max = 1000;
    def_options.in_engineering = false;
    def_options.debug = false;
    def_options.force_vector_rho = false; % If true, forces the penalty parameter rho to be defined as a vector
    
end
