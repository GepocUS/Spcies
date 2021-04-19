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
%   - recipe: An instance of the Spcies_problem class.
% 
% OUTPUTS:
%   - constructor: An instance of the Spcies_constructor class ready for file generation.
%                  
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function constructor = cons_laxMPC_ADMM_C(recipe)

    %% Preliminaries
    import utils.addLine

    % Get path to this directory
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    
    %% Default solver options
    def_solver_options.rho = 1e-2;
    def_solver_options.tol = 1e-4;
    def_solver_options.k_max = 1000;
    def_solver_options.in_engineering = false;
    def_solver_options.debug = false;
    def_solver_options.const_are_static = true;
    
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
    
    if solver_options.const_are_static
        const_type = 'static constant';
    else
        const_type = 'constant';
    end
    
    %% Create vars cell matrix: Name, value, initialize, type(int, float, etc), class(variable, constant, define, etc)
    
    % Defines
    defCell = [];
    defCell = addLine(defCell, 'nn', n, 1, 'uint', 'define');
    defCell = addLine(defCell, 'mm', m, 1, 'uint', 'define');
    defCell = addLine(defCell, 'nm', n+m, 1, 'uint', 'define');
    defCell = addLine(defCell, 'NN', N, 1, 'uint', 'define');
    defCell = addLine(defCell, 'k_max', solver_options.k_max, 1, 'uint', 'define');
    defCell = addLine(defCell, 'tol', solver_options.tol, 1, 'float', 'define');
    defCell = addLine(defCell, 'in_engineering', solver_options.in_engineering, 1, 'int', 'define');
    if solver_options.debug
        defCell = addLine(defCell, 'DEBUG', 1, 1, 'bool', 'define');
    end
    
    % Constants
    constCell = [];
    constCell = addLine(constCell, 'LB', vars.LB, 1, 'double', const_type);
    constCell = addLine(constCell, 'UB', vars.UB, 1, 'double', const_type);
    constCell = addLine(constCell, 'Hi', vars.Hi, 1, 'double', const_type);
    constCell = addLine(constCell, 'Hi_0', vars.Hi_0, 1, 'double', const_type);
    constCell = addLine(constCell, 'Hi_N', vars.Hi_N, 1, 'double', const_type);
    constCell = addLine(constCell, 'AB', vars.AB, 1, 'double', const_type);
    constCell = addLine(constCell, 'Alpha', vars.Alpha, 1, 'double', const_type);
    constCell = addLine(constCell, 'Beta', vars.Beta, 1, 'double', const_type);
    constCell = addLine(constCell, 'Q', vars.Q, 1, 'double', const_type);
    constCell = addLine(constCell, 'R', vars.R, 1, 'double', const_type);
    constCell = addLine(constCell, 'P', vars.P, 1, 'double', const_type);
    if solver_options.in_engineering
        constCell = addLine(constCell, 'scaling_x', vars.scaling_x, 1, 'double', const_type);
        constCell = addLine(constCell, 'scaling_u', vars.scaling_u, 1, 'double', const_type);
        constCell = addLine(constCell, 'scaling_i_u', vars.scaling_i_u, 1, 'double', const_type);
        constCell = addLine(constCell, 'OpPoint_x', vars.OpPoint_x, 1, 'double', const_type);
        constCell = addLine(constCell, 'OpPoint_u', vars.OpPoint_u, 1, 'double', const_type);
    end
    
    % rho
    if vars.rho_is_scalar
        defCell = addLine(defCell, 'SCALAR_RHO', 1, 0, 'bool', 'define');
        defCell = addLine(defCell, 'rho', vars.rho, 1, 'double', 'define');
        defCell = addLine(defCell, 'rho_i', vars.rho_i, 1, 'double', 'define');
    else
        constCell = addLine(constCell, 'rho', vars.rho, 1, 'double', const_type);
        constCell = addLine(constCell, 'rho_0', vars.rho_0, 1, 'double', const_type);
        constCell = addLine(constCell, 'rho_N', vars.rho_N, 1, 'double', const_type);
        constCell = addLine(constCell, 'rho_i', vars.rho_i, 1, 'double', const_type);
        constCell = addLine(constCell, 'rho_i_0', vars.rho_i_0, 1, 'double', const_type);
        constCell = addLine(constCell, 'rho_i_N', vars.rho_i_N, 1, 'double', const_type);
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

