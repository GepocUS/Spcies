%% def_options_MPCT_ADMM_cs
% 
% Returns the default options for the ADMM_cs-based solver for the MPCT formulation
% 
% Currently, there is no additional documentation available for the solver.
% 
% INPUT:
%   - submethod: string containing the submethod name (can be used for returning different default values)
% OUTPUT:
%   - def_options: Structure containing the default options of the solver
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function def_options = def_options_MPCT_ADMM(submethod)

    def_options.rho = 1e-2;
    def_options.epsilon_x = 1e-6;
    def_options.epsilon_u = 1e-6;
    def_options.tol = 1e-4;
    def_options.tol_p = 1e-4;
    def_options.tol_d = 1e-4;
    def_options.k_max = 1000;
    def_options.force_vector_rho = false; % If true, forces the penalty parameter rho to be defined as a vector
    
end
