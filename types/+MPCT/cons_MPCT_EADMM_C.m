%% cons_MPCT_EADMM_C
% 
% Generates the constructor for C of the EADMM-based solver for the MPCT formulation
% 
% Information about this formulation and the solver can be found at:
% 
% "Implementation of model predictive control for tracking in embedded systems
% using a sparse extended ADMM algorithm", by P. Krupa, I. Alvarado, D. Limon
% and T. Alamo, arXiv preprint: 2008:09071v2, 2020.
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
%       - solver_options: Structure containing options of the EADMM solver.
%              - .rho_base: Scalar. Base value of the penalty parameter.
%              - .rho_mult: Scalar. Multiplication factor of the base value.
%              - .epsilon_x: Vector by which the bound for x_s are reduced.
%              - .epsilon_u: Vector by which the bound for u_s are reduced.
%              - .inf_bound: Scalar. Determines the value given to components without bound.
%              - .tol: Exit tolerance of the solver. Defaults to 1e-4.
%              - .k_max: Maximum number of iterations of the solver. Defaults to 1000.
%              - .in_engineering: Boolean that determines if the arguments of the solver are given in
%                                 engineering units (true) or incremental ones (false - default).
%              - .debug: Boolean that determines if debugging options are enables in the solver.
%                        Defaults to false.
%              - .const_are_static: Boolean that determines if constants are defined as static variables.
%                                   Defaults to true.
%   - options: Structure containing the options of the toolbox. See Spcies_default_options.
% 
% OUTPUTS:
%   - constructor: An instance of the Spcies_constructor class ready for file generation.
%                  
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function constructor = cons_MPCT_EADMM_C(recipe)

    %% Preliminaries
    import utils.add_line

    % Get path to this directory
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    
    %% Default solver options
    def_solver_options.rho_base = 3;
    def_solver_options.rho_mult = 20;
    def_solver_options.epsilon_x = 1e-6;
    def_solver_options.epsilon_u = 1e-6;
    def_solver_options.inf_bound = 1e6;
    def_solver_options.tol = 1e-4;
    def_solver_options.k_max = 1000;
    def_solver_options.in_engineering = false;
    def_solver_options.debug = false;
    
    % Fill recipe.solver_options with the defaults
    solver_options = utils.add_default_options_to_struct(recipe.solver_options, def_solver_options);
    recipe.solver_options = solver_options;
    
    %% Compute the ingredients of the controller
    vars = MPCT.compute_MPCT_EADMM_ingredients(recipe.controller, solver_options, recipe.options);
    
    %% Set save_name to type if none is provided
    if isempty(recipe.options.save_name)
        recipe.options.save_name = recipe.options.type;
    end
    
    %% Rename variables for convenience
    n = vars.n;
    m = vars.m;
    N = vars.N;
    
    if recipe.options.const_are_static
        var_options = {'static', 'constant', 'array'};
    else
        var_options = {'constant', 'array'};
    end
    
    %% Create vars cell matrix: Name, value, initialize, type(int, float, etc), class(variable, constant, define, etc)

    % Defines
    defCell = [];
    defCell = add_line(defCell, 'nn', n, 1, 'uint', 'define');
    defCell = add_line(defCell, 'mm', m, 1, 'uint', 'define');
    defCell = add_line(defCell, 'nm', n+m, 1, 'uint', 'define');
    defCell = add_line(defCell, 'NN', N, 1, 'uint', 'define');
    defCell = add_line(defCell, 'k_max', solver_options.k_max, 1, 'uint', 'define');
    defCell = add_line(defCell, 'tol', solver_options.tol, 1, 'float', 'define');
    defCell = add_line(defCell, 'in_engineering', solver_options.in_engineering, 1, 'bool', 'define');
    if solver_options.debug
        defCell = add_line(defCell, 'DEBUG', 1, 1, 'bool', 'define');
    end
    
    % Constants
    constCell = [];
    constCell = add_line(constCell, 'rho', vars.rho, 1, 'float', var_options);
    constCell = add_line(constCell, 'rho_0', vars.rho_0, 1, 'float', var_options);
    constCell = add_line(constCell, 'rho_s', vars.rho_s, 1, 'float', var_options);
    constCell = add_line(constCell, 'LB', vars.LB, 1, 'float', var_options);
    constCell = add_line(constCell, 'UB', vars.UB, 1, 'float', var_options);
    constCell = add_line(constCell, 'LB_0', vars.LB0, 1, 'float', var_options);
    constCell = add_line(constCell, 'UB_0', vars.UB0, 1, 'float', var_options);
    constCell = add_line(constCell, 'LB_s', vars.LBs, 1, 'float', var_options);
    constCell = add_line(constCell, 'UB_s', vars.UBs, 1, 'float', var_options);
    constCell = add_line(constCell, 'AB', vars.AB, 1, 'float', var_options);
    constCell = add_line(constCell, 'T', vars.T, 1, 'float', var_options);
    constCell = add_line(constCell, 'S', vars.S, 1, 'float', var_options);
    constCell = add_line(constCell, 'Alpha', vars.Alpha, 1, 'float', var_options);
    constCell = add_line(constCell, 'Beta', vars.Beta, 1, 'float', var_options);
    constCell = add_line(constCell, 'H1i', vars.H1i, 1, 'float', var_options);
    constCell = add_line(constCell, 'W2', vars.W2, 1, 'float', var_options);
    constCell = add_line(constCell, 'H3i', vars.H3i, 1, 'float', var_options);
    if solver_options.in_engineering
        constCell = add_line(constCell, 'scaling_x', vars.scaling_x, 1, 'double', var_options);
        constCell = add_line(constCell, 'scaling_u', vars.scaling_u, 1, 'double', var_options);
        constCell = add_line(constCell, 'scaling_i_u', vars.scaling_i_u, 1, 'double', var_options);
        constCell = add_line(constCell, 'OpPoint_x', vars.OpPoint_x, 1, 'double', var_options);
        constCell = add_line(constCell, 'OpPoint_u', vars.OpPoint_u, 1, 'double', var_options);
    end

    %% Declare an empty constructor object
    constructor = Spcies_constructor;
    
    %% Fill in the files
    
    % .c file
    constructor = constructor.new_empty_file('code', recipe.options, 'c');
    constructor.files.code.blocks = {'$START$', C_code.get_generic_solver_struct;...
                                     '$INSERT_SOLVER$', [this_path '/code_MPCT_EADMM_C.c']};
      
    % .h file
    constructor = constructor.new_empty_file('header', recipe.options, 'h');
    constructor.files.header.blocks = {'$START$', [this_path '/header_MPCT_EADMM_C.h']};
    
    % Data
    constructor.data = {'$INSERT_DEFINES$', defCell;...
                        '$INSERT_CONSTANTS$', constCell};

end

