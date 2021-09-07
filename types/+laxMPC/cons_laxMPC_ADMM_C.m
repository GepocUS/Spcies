%% cons_laxMPC_ADMM_C
%
% Generates the constructor for C of the ADMM-based solver for the lax MPC formulation
%
% Information about this formulation and the solver can be found at:
%
% P. Krupa, D. Limon, T. Alamo, "Implementation of model predictive control in
% programmable logic controllers", Transactions on Control Systems Technology, 2020.
% 
% Specifically, this formulation is given in equation (9) of the above reference.
% 
% INPUTS:
%   - recipe: An instance of the Spcies_problem class. Its properties must contain:
%       - controller: Structure containing the information of the controller.
%                 It must contain the fields .sys and .param.
%                 - .sys: Structure containing the state space model (see Spcies_gen_controller).
%                 - .param: Structure containing the ingredients of the controller:
%                           - .Q: Cost function matrix Q.
%                           - .R: Cost function matrix R.
%                           - .P: Cost function matrix P.
%                           - .N: Prediction horizon.
%       - solver_options: Structure containing options of the ADMM solver.
%              - .rho: Penalty parameter. Scalar of vector. Defaults to the scalar 1e-2.
%                      If a vector is provided, it must have the same dimensions as the decision variables.
%              - .tol: Exit tolerance of the solver. Defaults to 1e-4.
%              - .k_max: Maximum number of iterations of the solver. Defaults to 1000.
%              - .in_engineering: Boolean that determines if the arguments of the solver are given in
%                                 engineering units (true) or incremental ones (false - default).
%              - .debug: Boolean that determines if debugging options are enables in the solver.
%                        Defaults to false.
%              - .const_are_static: Boolean that determines if constants are defined as static variables.
%                                   Defaults to true.
% 
% OUTPUTS:
%   - constructor: An instance of the Spcies_constructor class ready for file generation.
%                  
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function constructor = cons_laxMPC_ADMM_C(recipe)

    %% Preliminaries
    import utils.add_line

    % Get path to this directory
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    
    %% Default solver options
    def_solver_options = laxMPC.def_options_laxMPC_ADMM();
 
    % Fill recipe.solver_options with the defaults
    solver_options = utils.add_default_options_to_struct(recipe.solver_options, def_solver_options);
    recipe.solver_options = solver_options;
    
    %% Compute the ingredients of the controller
    vars = laxMPC.compute_laxMPC_ADMM_ingredients(recipe.controller, solver_options, recipe.options);
    
    %% Set save_name to type if none is provided
    if isempty(recipe.options.save_name)
        recipe.options.save_name = recipe.options.type;
    end
    
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
    defCell = [];
    defCell = add_line(defCell, 'nn', n, 1, 'uint', 'define');
    defCell = add_line(defCell, 'mm', m, 1, 'uint', 'define');
    defCell = add_line(defCell, 'nm', n+m, 1, 'uint', 'define');
    defCell = add_line(defCell, 'NN', N, 1, 'uint', 'define');
    defCell = add_line(defCell, 'k_max', solver_options.k_max, 1, 'uint', 'define');
    defCell = add_line(defCell, 'tol', solver_options.tol, 1, 'float', 'define');
    defCell = add_line(defCell, 'in_engineering', solver_options.in_engineering, 1, 'int', 'define');
    if solver_options.debug
        defCell = add_line(defCell, 'DEBUG', 1, 1, 'bool', 'define');
    end
%     if isdiag(vars.Q)
%         defCell = add_line(defCell, 'Q_IS_DIAG', 1, 1, 'bool', 'define');
%         vars.Q = diag(vars.Q);
%     end
%     if isdiag(vars.R)
%         defCell = add_line(defCell, 'R_IS_DIAG', 1, 1, 'bool', 'define');
%         vars.R = diag(vars.R);
%     end
%     if isdiag(vars.T)
%         defCell = add_line(defCell, 'T_IS_DIAG', 1, 1, 'bool', 'define');
%         vars.T = diag(vars.T);
%     end
    
    % Constants
    constCell = [];
    constCell = add_line(constCell, 'LB', vars.LB, 1, precision, var_options);
    constCell = add_line(constCell, 'UB', vars.UB, 1, precision, var_options);
%     constCell = add_line(constCell, 'Hi', vars.Hi, 1, precision, var_options);
%     constCell = add_line(constCell, 'Hi_0', vars.Hi_0, 1, precision, var_options);
%     constCell = add_line(constCell, 'Hi_N', vars.Hi_N, 1, precision, var_options);
    constCell = add_line(constCell, 'AB', vars.AB, 1, precision, var_options);
    constCell = add_line(constCell, 'GHi', vars.GHi, 1, precision, var_options);
    constCell = add_line(constCell, 'GHin', vars.GHin, 1, precision, var_options);
    constCell = add_line(constCell, 'GHiN', vars.GHiN, 1, precision, var_options);
    constCell = add_line(constCell, 'HiG', vars.HiG, 1, precision, var_options);
    constCell = add_line(constCell, 'HiGn', vars.HiGn, 1, precision, var_options);
    constCell = add_line(constCell, 'HiGN', vars.HiGN, 1, precision, var_options);
    constCell = add_line(constCell, 'Qi', vars.Qi, 1, precision, var_options);
    constCell = add_line(constCell, 'Ri', vars.Ri, 1, precision, var_options);
    constCell = add_line(constCell, 'Ti', vars.Ti, 1, precision, var_options);

    constCell = add_line(constCell, 'Alpha', vars.Alpha, 1, precision, var_options);
    constCell = add_line(constCell, 'Beta', vars.Beta, 1, precision, var_options);
    constCell = add_line(constCell, 'Q', vars.Q, 1, precision, var_options);
    constCell = add_line(constCell, 'R', vars.R, 1, precision, var_options);
    constCell = add_line(constCell, 'T', vars.T, 1, precision, var_options);
    if solver_options.in_engineering
        constCell = add_line(constCell, 'scaling_x', vars.scaling_x, 1, precision, var_options);
        constCell = add_line(constCell, 'scaling_u', vars.scaling_u, 1, precision, var_options);
        constCell = add_line(constCell, 'scaling_i_u', vars.scaling_i_u, 1, precision, var_options);
        constCell = add_line(constCell, 'OpPoint_x', vars.OpPoint_x, 1, precision, var_options);
        constCell = add_line(constCell, 'OpPoint_u', vars.OpPoint_u, 1, precision, var_options);
    end
    
    % rho
    if vars.rho_is_scalar
        defCell = add_line(defCell, 'SCALAR_RHO', 1, 0, 'bool', 'define');
        defCell = add_line(defCell, 'rho', vars.rho, 1, precision, 'define');
        defCell = add_line(defCell, 'rho_i', vars.rho_i, 1, precision, 'define');
    else
        error('Current version of SPCIES does not accept a non-scalar rho in the laxMPC ADMM solver');
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
                                     '$INSERT_SOLVER$', [this_path '/code_laxMPC_ADMM_C.c']};
      
    % .h file
    constructor = constructor.new_empty_file('header', recipe.options, 'h');
    constructor.files.header.blocks = {'$START$', [this_path '/header_laxMPC_ADMM_C.h']};
    
    % Data
    constructor.data = {'$INSERT_DEFINES$', defCell;...
                        '$INSERT_CONSTANTS$', constCell};

end

