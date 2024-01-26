%% def_options_ellipMPC_ADMM
% 
% Returns the default options for the ADMM-based solver for MPC with ellipsoidal terminal constraint.
% 
% Information about this formulation and the solver can be found at:
% 
% P. Krupa, R. Jaouani, D. Limon, and T. Alamo, “A sparse ADMM-based solver for linear MPC subject
% to terminal quadratic constraint,” arXiv:2105.08419, 2021.
%
% INPUT:
%   - submethod: string containing the submethod name (can be used for returning different default values)
% OUTPUT:
%   - def_options: Structure containing the default options of the solver.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function def_options = def_options_ellipMPC_ADMM(submethod)

    def_options.rho = 1e-2;
    def_options.tol = 1e-4;
    def_options.tol_p = 1e-4;
    def_options.tol_d = 1e-4;
    def_options.k_max = 1000;
    def_options.force_vector_rho = false; % If true, forces the penalty parameter rho to be defined as a vector
    
end
