% 
% Generates the constructor for C of the MPCT solver using ADMM on an extended state space
% 
% The solver extends the state and control inputs by adding the artificial reference to them.
% Currently, there is no additional documentation available for the solver.
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
%              - .rho: Penalty parameter. Scalar of vector. Defaults to the scalar 1e-2.
%                      If a vector is provided, it must have the same dimensions as the decision variables.
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
%   - options: Structure containing the options of the toolbox. See sp_utils.default_options.
% 
% OUTPUTS:
%   - constructor: An instance of the Spcies_constructor class ready for file generation.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function constructor = cons_MPCT_ADMM_cs_C(recipe)

    %% Preliminaries
    import sp_utils.add_line

    % Get path to this directory
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);

    %% Default solver options
    def_solver_options = MPCT.def_options_MPCT_ADMM_cs();
    
    % Fill recipe.solver_options with the defaults
    solver_options = sp_utils.add_default_options_to_struct(recipe.solver_options, def_solver_options);
    recipe.solver_options = solver_options;
    
    %% Compute the ingredients of the controller
    vars = MPCT.compute_MPCT_ADMM_cs_ingredients(recipe.controller, solver_options, recipe.options);
    
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
        if vars.rho_is_scalar
            var_options_rho = {'static', 'constant'};
        end
    else
        var_options = {'constant', 'array'};
        if vars.rho_is_scalar
            var_options_rho = {'constant'};
        end
    end
    
    % Determine if float or double variables are used
    precision = recipe.options.precision;
    
    %% Create vars cell matrix: Name, value, initialize, type(int, float, etc), class(variable, constant, define, etc)
    
    % Defines
    defCell = [];
    defCell = add_line(defCell, 'nn_', n, 1, 'uint', 'define');
    defCell = add_line(defCell, 'mm_', m, 1, 'uint', 'define');
    defCell = add_line(defCell, 'nm_', n+m, 1, 'uint', 'define');
    defCell = add_line(defCell, 'dnm_', 2*(n+m), 1, 'uint', 'define');
    defCell = add_line(defCell, 'nrow_AHi', vars.AHi_CSR.nrow, 1, 'uint', 'define');
    defCell = add_line(defCell, 'nrow_HiA', vars.HiA_CSR.nrow, 1, 'uint', 'define');
    defCell = add_line(defCell, 'NN_', N, 1, 'uint', 'define');
    defCell = add_line(defCell, 'k_max', solver_options.k_max, 1, 'uint', 'define');
    defCell = add_line(defCell, 'tol', solver_options.tol, 1, precision, 'define');
    defCell = add_line(defCell, 'in_engineering', solver_options.in_engineering, 1, 'bool', 'define');
    if solver_options.debug
        defCell = add_line(defCell, 'DEBUG', 1, 1, 'bool', 'define');
    end
    if recipe.options.time
        defCell = add_line(defCell, 'MEASURE_TIME', 1, 1, 'bool', 'define');
    end
    if vars.rho_is_scalar
        defCell = add_line(defCell, 'SCALAR_RHO', 1, 0, 'bool', 'define');
    end
    
    % Constants
    constCell = [];
    constCell = add_line(constCell, 'rho', vars.rho, 1, precision, var_options_rho);
    constCell = add_line(constCell, 'rho_i', vars.rho_i, 1, precision, var_options_rho);
    constCell = add_line(constCell, 'Tz', vars.Tz, 1, precision, var_options);
    constCell = add_line(constCell, 'Sz', vars.Sz, 1, precision, var_options);
    constCell = add_line(constCell, 'LB', vars.LB, 1, precision, var_options);
    constCell = add_line(constCell, 'UB', vars.UB, 1, precision, var_options);
    constCell = add_line(constCell, 'L_val', vars.L_CSC.val, 1, precision, var_options);
    constCell = add_line(constCell, 'L_col', vars.L_CSC.col-1, 1, 'int', var_options);
    constCell = add_line(constCell, 'L_row', vars.L_CSC.row-1, 1, 'int', var_options);
    constCell = add_line(constCell, 'Dinv', vars.Dinv, 1, precision, var_options);
    constCell = add_line(constCell, 'AHi_val', vars.AHi_CSR.val, 1, precision, var_options);
    constCell = add_line(constCell, 'AHi_col', vars.AHi_CSR.col-1, 1, 'int', var_options);
    constCell = add_line(constCell, 'AHi_row', vars.AHi_CSR.row-1, 1, 'int', var_options);
    constCell = add_line(constCell, 'HiA_val', vars.HiA_CSR.val, 1, precision, var_options);
    constCell = add_line(constCell, 'HiA_col', vars.HiA_CSR.col-1, 1, 'int', var_options);
    constCell = add_line(constCell, 'HiA_row', vars.HiA_CSR.row-1, 1, 'int', var_options);
    constCell = add_line(constCell, 'Hi_val', vars.Hi_CSR.val, 1, precision, var_options);
    constCell = add_line(constCell, 'Hi_col', vars.Hi_CSR.col-1, 1, 'int', var_options);
    constCell = add_line(constCell, 'Hi_row', vars.Hi_CSR.row-1, 1, 'int', var_options);
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
                                     '$INSERT_SOLVER$', [this_path '/code_MPCT_ADMM_cs_C.c']};
      
    % .h file
    constructor = constructor.new_empty_file('header', recipe.options, 'h');
    constructor.files.header.blocks = {'$START$', [this_path '/header_MPCT_ADMM_cs_C.h']};
    
    % Data
    constructor.data = {'$INSERT_DEFINES$', defCell;...
                        '$INSERT_CONSTANTS$', constCell};

end

