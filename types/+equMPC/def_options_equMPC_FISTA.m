%% def_options_equMPC_FISTA
% 
% Returns the default options for the FISTA-based solver for the equality MPC formulation.
% 
% Information about this formulation and the solver can be found at:
% 
% P. Krupa, D. Limon, T. Alamo, "Implementation of model predictive control in
% programmable logic controllers", Transactions on Control Systems Technology, 2020.
%
% OUTPUT:
%   - def_options: Structure containing the default options of the solver.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function def_options = def_options_equMPC_FISTA()

    def_options.tol = 1e-4; % Tolerance of the optimization algorithm for the exit condition
    def_options.k_max = 1000; % Maximum number of iterations allowed for the optimization algorithm
    def_options.in_engineering = false; % Selects whether the input variables are incremental or in engineering units
    def_options.debug = false; % If true, a structure with selected information about the optimization problem result is returned
    def_options.time_varying = false;  %When true, the generated MPC solver function accepts A, B, Q and R matrices as input parameters
    
end
