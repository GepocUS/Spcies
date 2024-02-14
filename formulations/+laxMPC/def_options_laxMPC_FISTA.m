%% def_options_laxMPC_FISTA
% 
% Returns the default options for the FISTA-based solver for the lax MPC formulation.
% 
% Information about this formulation and the solver can be found at:
% 
% P. Krupa, D. Limon, T. Alamo, "Implementation of model predictive control in
% programmable logic controllers", Transactions on Control Systems Technology, 2020.
%
% INPUT:
%   - submethod: string containing the submethod name (can be used for returning different default values)
% OUTPUT:
%   - def_options: Structure containing the default options of the solver.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function def_options = def_options_laxMPC_FISTA(submethod)

    def_options.tol = 1e-4; % Tolerance of the optimization algorithm for the exit condition
    def_options.k_max = 1000; % Maximum number of iterations allowed for the optimization algorithm
    
end
