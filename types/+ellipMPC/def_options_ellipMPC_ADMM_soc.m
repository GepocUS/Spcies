%% def_options_ellipMPC_ADMM_soc
% 
% Returns the default options for the ADMM-based solver for MPC with ellipsoidal terminal constraint
% using a SOC to impose the ellipsoidal constraint.
% 
% The ellipMPC formulation can be found at 
% 
% P. Krupa, R. Jaouani, D. Limon, and T. Alamo, “A sparse ADMM-based solver for linear MPC subject
% to terminal quadratic constraint,” arXiv:2105.08419, 2021.
% 
% However, there is currently no specific documentation on this solver.
% 
% OUTPUT:
%   - def_options: Structure containing the default options of the solver.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function def_options = def_options_ellipMPC_ADMM_soc()
    
    def_options.rho = 1e-2;
    def_options.tol_p = 1e-4;
    def_options.tol_d = 1e-4;
    def_options.k_max = 1000;
    def_options.in_engineering = false;
    def_options.debug = false;
    
end
