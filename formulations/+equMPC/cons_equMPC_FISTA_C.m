%% cons_equMPC_FISTA_C
%
% Generates the constructor for C of the FISTA-based solver for the equality MPC formulation
%
% Information about this formulation and the solver can be found at:
%
% P. Krupa, D. Limon, T. Alamo, "Implementation of model predictive control in
% programmable logic controllers", Transactions on Control Systems Technology, 2020.
% 
% Specifically, this formulation is given in equation (8) of the above reference.
% 
% INPUTS:
%   - recipe: An instance of the Spcies_problem class. Its properties must contain:
%       - controller: Structure containing the information of the controller.
%                 It must contain the fields .sys and .param.
%                 - .sys: Structure containing the state space model (see Spcies_gen_controller).
%                 - .param: Structure containing the ingredients of the controller:
%                           - .Q: Cost function matrix Q.
%                           - .R: Cost function matrix R.
%                           - .N: Prediction horizon.
%       - solver_options: Structure containing options of the FISTA solver.
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

function constructor = cons_equMPC_FISTA_C(recipe)

    %% Preliminaries
    import sp_utils.add_line

    % Get path to this directory
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    
    %% Default solver options
    def_solver_options = equMPC.def_options_equMPC_FISTA();
    
    % Fill recipe.solver_options with the defaults
    solver_options = sp_utils.add_default_options_to_struct(recipe.solver_options, def_solver_options);
    recipe.solver_options = solver_options;
    
    %% Compute the ingredients of the controller
    vars = equMPC.compute_equMPC_FISTA_ingredients(recipe.controller, solver_options, recipe.options);

    % Check that the options are allowed
    if solver_options.time_varying && size(vars.LB, 2) > 1
        error("EquMPC FISTA time varying solver only allows fixed bounds along the prediction horizon");
    end
    
    %% Set save_name to formulation if none is provided
    if isempty(recipe.options.save_name)
        recipe.options.save_name = recipe.options.formulation;
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
    defCell = add_line(defCell, 'nn_', n, 1, 'uint', 'define');
    defCell = add_line(defCell, 'mm_', m, 1, 'uint', 'define');
    defCell = add_line(defCell, 'nm_', n+m, 1, 'uint', 'define');
    defCell = add_line(defCell, 'NN_', N, 1, 'uint', 'define');
    defCell = add_line(defCell, 'k_max', solver_options.k_max, 1, 'uint', 'define');
    defCell = add_line(defCell, 'tol', solver_options.tol, 1, 'float', 'define');
    defCell = add_line(defCell, 'in_engineering', solver_options.in_engineering, 1, 'int', 'define');
    defCell= add_line(defCell, 'TIME_VARYING', solver_options.time_varying, 1, 'int', 'define');
    if solver_options.debug
        defCell = add_line(defCell, 'DEBUG', 1, 1, 'bool', 'define');
    end
    if recipe.options.time
        defCell = add_line(defCell, 'MEASURE_TIME', 1, 1, 'bool', 'define');
    end
    
    % Constants
    constCell = [];
    if size(vars.LB, 2) > 1
        % Different constraints for each prediction step
        constCell = add_line(constCell, 'LB0', vars.LB(n+1:end, 1), 1, precision, var_options);
        constCell = add_line(constCell, 'UB0', vars.UB(n+1:end, 1), 1, precision, var_options);
        constCell = add_line(constCell, 'LB', vars.LB(:, 2:end)', 1, precision, var_options);
        constCell = add_line(constCell, 'UB', vars.UB(:, 2:end)', 1, precision, var_options);
        defCell = add_line(defCell, 'VAR_BOUNDS', 1, 1, 'int', 'define');
    else
        if ~solver_options.time_varying
            constCell = add_line(constCell, 'LB', vars.LB, 1, precision, var_options);
            constCell = add_line(constCell, 'UB', vars.UB, 1, precision, var_options);
        end
    end
    if ~solver_options.time_varying
        constCell = add_line(constCell, 'AB', vars.AB, 1, precision, var_options);
        constCell = add_line(constCell, 'Alpha', vars.Alpha, 1, precision, var_options);
        constCell = add_line(constCell, 'Beta', vars.Beta, 1, precision, var_options);
        constCell = add_line(constCell, 'Q', vars.Q, 1, precision, var_options);
        constCell = add_line(constCell, 'R', vars.R, 1, precision, var_options);
        constCell = add_line(constCell, 'QRi', vars.QRi, 1, precision, var_options);
    end
    if solver_options.in_engineering
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
    constructor = constructor.new_empty_file('code', recipe.options, 'c');
    constructor.files.code.blocks = {'$START$', C_code.get_generic_solver_struct;...
                                     '$INSERT_SOLVER$', [this_path '/code_equMPC_FISTA_C.c']};
      
    % .h file
    constructor = constructor.new_empty_file('header', recipe.options, 'h');
    constructor.files.header.blocks = {'$START$', [this_path '/header_equMPC_FISTA_C.h']};
    
    % Data
    constructor.data = {'$INSERT_DEFINES$', defCell;...
                        '$INSERT_CONSTANTS$', constCell};

end
