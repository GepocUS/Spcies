%% def_options_MPCT_EADMM
% 
% Returns the default options for the EADMM-based solver for the MPCT formulation
% 
% Information about this formulation and the solver can be found at:
% 
% "Implementation of model predictive control for tracking in embedded systems
% using a sparse extended ADMM algorithm", by P. Krupa, I. Alvarado, D. Limon
% and T. Alamo, arXiv preprint: 2008:09071v2, 2020.
% 
% INPUT:
%   - submethod: string containing the submethod name (can be used for returning different default values)
% OUTPUT:
%   - def_options: Structure containing the default options of the solver
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function def_options = def_options_MPCT_EADMM(submethod)

    def_options.rho_base = 3;
    def_options.rho_mult = 20;
    def_options.epsilon_x = 1e-6;
    def_options.epsilon_u = 1e-6;
    def_options.tol = 1e-4;
    def_options.k_max = 1000;

end
