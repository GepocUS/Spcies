%% cons_ellipMPC_ADMM_C
%
% Generates the constructor for C of the ADMM-based solver for MPC with ellipsoidal terminal constraint
%
% Information about this formulation and the solver can be found at:
% 
% P. Krupa, R. Jaouani, D. Limon, and T. Alamo, “A sparse ADMM-based solver for linear MPC subject
% to terminal quadratic constraint,” arXiv:2105.08419, 2021.
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
%                           - .N: Prediction horizon.
%                           - .P: Matrix defining the geometry of the ellipsoidal terminal constraint.
%                           - .c: Matrix defining the center of the ellipsoidal terminal constraint.
%                                 Defaults to a vector of zeros (i.e., the origin).
%                           - .r: Scalar defining the size of the ellipsoidal terminal constraint.
%                                 Defaults to 1.
%       - options: Instance of Spcies_options. Solver specific options are:
%              - .rho: Penalty parameter. Scalar of vector.
%                      If a vector is provided, it must have the same dimensions as the decision variables.
%              - .tol: Exit tolerance of the solver.
%              - .k_max: Maximum number of iterations of the solver.
% 
% OUTPUTS:
%   - constructor: An instance of the Spcies_constructor class ready for file generation.
%                  
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function constructor = cons_ellipMPC_ADMM_C(recipe)

    %% Preliminaries
    import sp_utils.add_line

    % Get path to this directory
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    
    %% Compute the ingredients of the controller
    vars = ellipMPC.compute_ellipMPC_ADMM_ingredients(recipe.controller, recipe.options);
    
    %% Rename variables for convenience
    n = vars.n;
    m = vars.m;
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
    defCell = add_line(defCell, 'nn_', n, 1, 'uint', 'define');
    defCell = add_line(defCell, 'mm_', m, 1, 'uint', 'define');
    defCell= add_line(defCell, 'nm_', n+m, 1, 'uint', 'define');
    defCell = add_line(defCell, 'NN_', N, 1, 'uint', 'define');
    defCell = add_line(defCell, 'k_max', recipe.options.solver.k_max, 1, 'uint', 'define');
    defCell= add_line(defCell, 'tol', recipe.options.solver.tol, 1, 'float', 'define');
    
    % Constants
    constCell = [];
    constCell = add_line(constCell, 'LBu0', vars.LBu0, 1, precision, var_options);
    constCell = add_line(constCell, 'UBu0', vars.UBu0, 1, precision, var_options);
    constCell = add_line(constCell, 'LBz', vars.LBz, 1, precision, var_options);
    constCell = add_line(constCell, 'UBz', vars.UBz, 1, precision, var_options);
    constCell = add_line(constCell, 'Hi', vars.Hi, 1, precision, var_options);
    constCell = add_line(constCell, 'Hi_0', vars.Hi_0, 1, precision, var_options);
    constCell = add_line(constCell, 'Hi_N', vars.Hi_N, 1, precision, var_options);
    constCell = add_line(constCell, 'AB', vars.AB, 1, precision, var_options);
    constCell = add_line(constCell, 'P', vars.P, 1, precision, var_options);
    constCell = add_line(constCell, 'P_half', vars.P_half, 1, precision, var_options);
    constCell = add_line(constCell, 'Pinv_half', vars.Pinv_half, 1, precision, var_options);
    constCell = add_line(constCell, 'Alpha', vars.Alpha, 1, precision, var_options);
    constCell = add_line(constCell, 'Beta', vars.Beta, 1, precision, var_options);
    constCell= add_line(constCell, 'Q', vars.Q, 1, precision, var_options);
    constCell = add_line(constCell, 'R', vars.R, 1, precision, var_options);
    constCell = add_line(constCell, 'T', vars.T, 1, precision, var_options);
    if recipe.options.in_engineering
        constCell = add_line(constCell, idx, 'scaling_x', vars.scaling_x, 1, precision, var_options);
        constCell = add_line(constCell, idx, 'scaling_u', vars.scaling_u, 1, precision, var_options);
        constCell = add_line(constCell, idx, 'scaling_i_u', vars.scaling_i_u, 1, precision, var_options);
        constCell = add_line(constCell, idx, 'OpPoint_x', vars.OpPoint_x, 1, precision, var_options);
        constCell = add_line(constCell, idx, 'OpPoint_u', vars.OpPoint_u, 1, precision, var_options);
    end
    
    % Variables
    varsCell = [];
    varsCell = add_line(varsCell, 'c', vars.c, 1, precision, 'variable');
    varsCell = add_line(varsCell, 'r', vars.r, 1, precision, 'variable');
    
    % rho
    if vars.rho_is_scalar
        defCell = add_line(defCell, 'SCALAR_RHO', 1, 0, 'bool', 'define');
        defCell = add_line(defCell, 'rho', vars.rho, 1, precision, 'define');
        defCell = add_line(defCell, 'rho_i', vars.rho_i, 1, precision, 'define');
    else
        constCell = add_line(constCell, 'rho', vars.rho, 1, precision, var_options);
        constCell = add_line(constCell, 'rho_0', vars.rho_0, 1, precision, var_options);
        constCell = add_line(constCell, 'rho_N', vars.rho_N, 1, precision, var_options);
        constCell = add_line(constCell, 'rho_i', vars.rho_i, 1, precision, var_options);
        constCell = add_line(constCell, 'rho_i_0', vars.rho_i_0, 1, precision, var_options);
        constCell = add_line(constCell, 'rho_i_N', vars.rho_i_N, 1, precision, var_options);
    end

    %% Declare an empty constructor object
    constructor = Spcies_constructor;
    
    %% Fill in the files
    
    % .c file
    constructor = constructor.new_empty_file('code', recipe.options, 'c');
    constructor.files.code.blocks = {'$START$', C_code.get_generic_solver_struct;...
                                     '$INSERT_SOLVER$', [this_path '/code_ellipMPC_ADMM_C.c']};
      
    % .h file
    constructor = constructor.new_empty_file('header', recipe.options, 'h');
    constructor.files.header.blocks = {'$START$', [this_path '/header_ellipMPC_ADMM_C.h']};
    
    % Data
    constructor.data = {'$INSERT_DEFINES$', defCell;...
                        '$INSERT_CONSTANTS$', constCell;...
                        '$INSERT_VARIABLES$', varsCell};

end
