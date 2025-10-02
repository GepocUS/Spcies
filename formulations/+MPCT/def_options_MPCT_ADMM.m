%% def_options_MPCT_ADMM
% 
% Returns the default options for the ADMM-based solvers for MPCT
% Submethods:
%   - 'semiband': semibanded solver, see compute_MPCT_ADMM_semiband_ingredients()
%   - 'cs': No additional documentation for the solver
% 
% 
% INPUT:
%   - submethod: string containing the submethod name (can be used for returning different default values)
% OUTPUT:
%   - def_options: Structure containing the default options of the solver
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function def_options = def_options_MPCT_ADMM(submethod)

    switch submethod

        case 'cs'
            def_options.rho = 1e-2;
            def_options.epsilon_x = 1e-6;
            def_options.epsilon_u = 1e-6;
            def_options.tol = 1e-4;
            def_options.tol_p = 1e-4;
            def_options.tol_d = 1e-4;
            def_options.k_max = 1000;
            def_options.force_vector_rho = false; % If true, forces the penalty parameter rho to be defined as a vector
        
        case 'semiband'
            def_options.rho = 1e-2;
            def_options.epsilon_x = 1e-6;
            def_options.epsilon_u = 1e-6;
            def_options.epsilon_y = 1e-6; % Only useful if constrained_output is activated along with hard constraints
            def_options.tol_p = 1e-4;
            def_options.tol_d = 1e-4;
            def_options.k_max = 1000;
            def_options.force_vector_rho = false; % If true, forces the penalty parameter rho to be defined as a vector.
            def_options.initialize_iterates = false; % If true, initial values for v (primal variables) and lambda (dual variables) must be given.
            def_options.soft_constraints = false; % If true, soft constraints are allowed.
            def_options.constrained_output = false; % If true, contraints of kind LB<= C*x+D*u <= UB are allowed.
            % Also, every inequality constraint is soft constrained except for the ones in u_0.
            def_options.force_vector_beta = false; % When soft_constraints == true and adaptive_beta == false, forces internally the penalization of soft constraints beta to be a vector.
            def_options.adaptive_beta = false; % If true, weights for soft constraints can change online.
            def_options.adaptive_beta_is_vector = true; % If true, when adaptive_beta is set to true, the solver requires a vector for beta. If false, the solver requires a scalar.
            def_options.beta = 1; % Only useful if soft constraints are set to true.
        
        otherwise
                error("spcies: MPCT submethod '" + submethod + "' not recognized");  

    end
end
