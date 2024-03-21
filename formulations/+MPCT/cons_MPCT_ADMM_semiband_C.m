%% cons_MPCT_ADMM_semiband_C
% 
% Generates the constructor for C of the ADMM_semiband-based solver for the MPCT formulation
% 
% Information about this formulation and the solver can be found at:
% 
% TODO: Insert cite of the article if published
%
% INPUTS:
%   - recipe: An instance of the Spcies_problem class. Its properties must contain:
%       - controller: Structure containing the information of the controller.
%                 It must contain the fields .sys and .param.
%                 - .sys: Structure containing the state space model (see Spcies_gen_controller).
%                 - .param: Structure containing the ingredients of the controller:
%                           - .Q: Cost function matrix Q.
%                           - .R: Cost function matrix R.
%                           - .T: Cost function matrix T.
%                           - .S: Cost function matrix S.
%                           - .N: Prediction horizon.
%       - options: Instance of Spcies_options. Solver specific options are:
%              - .rho: Scalar. Value of the penalty parameter.
%              - .epsilon_x: Vector by which the bound for x_s are reduced.
%              - .epsilon_u: Vector by which the bound for u_s are reduced.
%              - .epsilon_y: Vector by which the bound for C*x_s+D*u_s is reduced.
%              - .inf_bound: Scalar. Determines the value given to components without bound.
%              - .tol: Exit tolerance of the solver.
%              - .k_max: Maximum number of iterations of the solver.
%              - .soft_constraints: Determines if soft constraints are allowed.
%              - .constrained_output: Determines if constraints of kind LBy <= C*x+D*u <= UBy are allowed.
% OUTPUTS:
%   - constructor: An instance of the Spcies_constructor class ready for file generation.
%                  
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 


function constructor = cons_MPCT_ADMM_semiband_C(recipe)

    %% Preliminaries
    import sp_utils.add_line

    % Get path to this directory
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);

    %% Compute the ingredients of the controller
    vars = MPCT.compute_MPCT_ADMM_semiband_ingredients(recipe.controller, recipe.options);

    % Detect if weight matrices are diagonal
    if (isdiag(vars.Q) && isdiag(vars.R) && isdiag(vars.T) && isdiag(vars.S))
        recipe.options.force_diagonal = true; % This option does nothing in this solver for now
    else
        recipe.options.force_diagonal = false;
    end

    %% Rename variables for convenience
    n = vars.n;
    m = vars.m;
    if recipe.options.solver.constrained_output
        p = vars.p;
    end
    N = vars.N;

    % Determine if constant variables are defined as static
    if recipe.options.const_are_static
        var_options = {'static', 'constant', 'array'};
    else
        var_options = {'constant', 'array'};
    end
    
    % Determine if float or double variables are used
    precision = recipe.options.precision;

    %% Create vars cell matrix: Name, value, initialize, type(int, float, etc), class(variable, constant, define, etc)

    % Defines
    defCell = recipe.options.default_defCell();
    defCell = add_line(defCell, 'SOFT_CONSTRAINTS', recipe.options.solver.soft_constraints, 1, 'bool', 'define');
    defCell = add_line(defCell, 'CONSTRAINED_OUTPUT', recipe.options.solver.constrained_output, 1, 'bool', 'define');
    defCell = add_line(defCell, 'nn_', n, 1, 'uint', 'define');
    defCell = add_line(defCell, 'mm_', m, 1, 'uint', 'define');
    defCell = add_line(defCell, 'nm_', n+m, 1, 'uint', 'define');
    if recipe.options.solver.constrained_output
        defCell = add_line(defCell, 'pp_', p, 1, 'uint', 'define');
        defCell = add_line(defCell, 'nmp_', n+m+p, 1, 'uint', 'define');
    end
    defCell = add_line(defCell, 'NN_', N, 1, 'uint', 'define');
    defCell = add_line(defCell, 'k_max', recipe.options.solver.k_max, 1, 'uint', 'define');
    defCell = add_line(defCell, 'tol_p', recipe.options.solver.tol_p, 1, 'float', 'define');
    defCell = add_line(defCell, 'tol_d', recipe.options.solver.tol_d, 1, 'float', 'define');
    if ~recipe.options.solver.soft_constraints
        defCell = add_line(defCell, 'eps_x', recipe.options.solver.epsilon_x, 1, 'float', 'define');
        defCell = add_line(defCell, 'eps_u', recipe.options.solver.epsilon_u, 1, 'float', 'define');
        if recipe.options.solver.constrained_output
            defCell = add_line(defCell, 'eps_y', recipe.options.solver.epsilon_y, 1, 'float', 'define');
        end
    end
    defCell = add_line(defCell, 'inf', recipe.options.inf_value, 1, 'float', 'define');
    
    % Constants
    constCell = [];

    if vars.rho_is_scalar
        defCell = add_line(defCell, 'SCALAR_RHO', 1, 0, 'bool', 'define');
        defCell = add_line(defCell, 'rho', vars.rho, 1, precision, 'define');
        defCell = add_line(defCell, 'rho_i', vars.rho_i, 1, precision, 'define');
        if recipe.options.solver.soft_constraints
            defCell = add_line(defCell, 'beta_rho_i', vars.beta_rho_i, 1, precision, 'define');
        end
    else
        constCell = add_line(constCell, 'rho', vars.rho, 1, precision, var_options);
        constCell = add_line(constCell, 'rho_i', vars.rho_i, 1, precision, var_options);
        if recipe.options.solver.soft_constraints
            constCell = add_line(constCell, 'beta_rho_i', vars.beta_rho_i, 1, precision, var_options);
        end
    end
    constCell = add_line(constCell, 'LB', vars.LB, 1, precision, var_options);
    constCell = add_line(constCell, 'UB', vars.UB, 1, precision, var_options);
    constCell = add_line(constCell, 'Q', vars.Q, 1, precision, var_options);
    constCell = add_line(constCell, 'R', vars.R, 1, precision, var_options);
    constCell = add_line(constCell, 'S', vars.S, 1, precision, var_options);
    constCell = add_line(constCell, 'T', vars.T, 1, precision, var_options);
    constCell = add_line(constCell, 'Q_rho_i', vars.Q_rho_i, 1, precision, var_options);
    constCell = add_line(constCell, 'R_rho_i', vars.R_rho_i, 1, precision, var_options);
    constCell = add_line(constCell, 'S_rho_i', vars.S_rho_i, 1, precision, var_options);
    constCell = add_line(constCell, 'T_rho_i', vars.T_rho_i, 1, precision, var_options);
    constCell = add_line(constCell, 'A', vars.A, 1, precision, var_options);
    constCell = add_line(constCell, 'B', vars.B, 1, precision, var_options);
    if recipe.options.solver.constrained_output
        var_options_CD = var_options;
        if size(vars.C,1) == 1
            var_options_CD = [var_options,'matrix']; % TODO: This still neeeds a better solution
        end
        constCell = add_line(constCell, 'CD', [vars.C,vars.D], 1, precision, var_options_CD);
    end
    constCell = add_line(constCell, 'Alpha', vars.Alpha, 1, precision, var_options);
    constCell = add_line(constCell, 'Beta', vars.Beta, 1, precision, var_options);
    constCell = add_line(constCell, 'U_tilde', vars.U_tilde, 1, precision, var_options);
    constCell = add_line(constCell, 'M_hat_x1', vars.M_hat_x1, 1, precision, var_options);
    constCell = add_line(constCell, 'M_hat_x2', vars.M_hat_x2, 1, precision, var_options);
    constCell = add_line(constCell, 'M_hat_u1', vars.M_hat_u1, 1, precision, var_options);
    constCell = add_line(constCell, 'M_hat_u2', vars.M_hat_u2, 1, precision, var_options);
    constCell = add_line(constCell, 'M_tilde', vars.M_tilde, 1, precision, var_options);
    

    if recipe.options.in_engineering
        constCell = add_line(constCell, 'scaling_x', vars.scaling_x, 1, precision, var_options);
        constCell = add_line(constCell, 'scaling_u', vars.scaling_u, 1, precision, var_options);
        constCell = add_line(constCell, 'scaling_i_u', vars.scaling_i_u, 1, precision, var_options);
        constCell = add_line(constCell, 'OpPoint_x', vars.OpPoint_x, 1, precision, var_options);
        constCell = add_line(constCell, 'OpPoint_u', vars.OpPoint_u, 1, precision, var_options);
    end

    %% Declare an empty constructor object
    constructor = Spcies_constructor;

    %% Fill in the files

    % .c file
    constructor = constructor.new_empty_file('code',recipe.options,'c');
    constructor.files.code.blocks = {'$START$', C_code.get_generic_solver_struct;...
                                     '$INSERT_SOLVER$', [this_path '/code_MPCT_ADMM_semiband_C.c']};

    % .h file
    constructor = constructor.new_empty_file('header',recipe.options,'h');
    constructor.files.header.blocks = {'$START$', [this_path '/header_MPCT_ADMM_semiband_C.h']};

    % Data
    constructor.data = {'$INSERT_DEFINES$', defCell;...
                        '$INSERT_CONSTANTS$', constCell};

end

