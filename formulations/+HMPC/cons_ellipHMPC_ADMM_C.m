%% cons_ellipHMPC_ADMM_C
%
% Generates the constructor for C of the HMPC formulation based on ADMM with harmonic reference.
% 
% Information about this formulation can be found at:
%
% P. Krupa, D. Limon, and T. Alamo, “Harmonic based model predictive
% control for set-point tracking", IEEE Transactions on Automatic Control.
%
% Information about the solver canbe found in:
%
% Pablo Krupa, Daniel Limon, Alberto Bemporad, Teodoro Alamo, "Efficiently
% solving the hamonic model predictive control formulation", arXiv: 2202.06629, 2022.
% 
% INPUTS:
%   - recipe: An instance of the Spcies_problem class. Its properties must contain:
%       - controller: Structure containing the information of the controller.
%                 It must contain the fields .sys and .param.
%                 - .sys: Structure containing the state space model (see Spcies_gen_controller).
%                 - .param: Structure containing the ingredients of the controller:
%                           - .Q: Cost function matrix Q.
%                           - .R: Cost function matrix R.
%                           - .Te: Cost function matrix Te.
%                           - .Th: Cost function matrix Th.
%                           - .Se: Cost function matrix Se.
%                           - .Sh: Cost function matrix Sh.
%                           - .N: Prediction horizon.
%                           - .w: Base frequency.
%       - solver_options: Structure containing options of the solver.
%                         Default values provided in def_options_HMPC_SADMM.m
%              - .sigma: Penalty parameter sigma. Scalar.
%              - .rho: Penalty parameter rho. Scalar.
%              - .alpha: Relaxation parameter of the SADMM algorithm.
%              - .tol_p: Primal exit tolerance of the solver.
%              - .tol_d: Dual exit tolerance of the solver.
%              - .k_max: Maximum number of iterations of the solver.
%              - .in_engineering: Boolean that determines if the arguments of the solver are given in
%                                 engineering units (true) or incremental ones (false - default).
%              - .debug: Boolean that determines if debugging options are enables in the solver.
%                        Defaults to false.
%              - sparse: Boolean that determines if the system os equations is solved using the sparse
%                        LDL approach (if true) or the non-sparse approach (if false).
%              - use_soc: Boolean that determines if the SOC constraints are grouped into the "diamond"
%                         sets (if false) or if they are all considered (if true).
%              - .const_are_static: Boolean that determines if constants are defined as static variables.
%                                   Defaults to true.
% 
% OUTPUTS:
%   - constructor: An instance of the Spcies_constructor class ready for file generation.
%                  
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function constructor = cons_HMPC_ADMM_C(recipe)

    %% Preliminaries
    import sp_utils.add_line

    % Get path to this directory
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);

    %% Default solver options
    def_solver_options = HMPC.def_options_ellipHMPC_ADMM();
    
    % Fill recipe.solver_options with the defaults
    solver_options = sp_utils.add_default_options_to_struct(recipe.solver_options, def_solver_options);
    recipe.solver_options = solver_options;
    
    %% Determine which solver to use: box-constrained or coupled-inputs
    if isempty(recipe.solver_options.box_constraints)
        if isfield(recipe.controller.sys, 'E')
            recipe.solver_options.box_constraints = false;
        else
            recipe.solver_options.box_constraints = true;
        end
    end
    
    %% Compute the ingredients of the controller
    vars = HMPC.compute_ellipHMPC_ADMM_ingredients(recipe.controller, solver_options, recipe.options);
    
    %% Set save_name to formulation if none is provided
    if isempty(recipe.options.save_name)
        recipe.options.save_name = recipe.options.formulation;
    end

    %% Rename variables for convenience
    n = vars.n;
    m = vars.m;
    N = vars.N;
    dim = vars.dim;
    n_s = vars.n_s;
    n_eq = vars.n_eq;
    n_y = vars.n_y;
    n_soc = vars.n_soc;
    n_box = vars.n_box;

    % Determine if constant variables are defined as static
    if recipe.options.const_are_static
        var_options = {'static', 'constant', 'array'};  
        var_options_penalty = {'static', 'constant'};
    else
        var_options = {'constant', 'array'};
        var_options_penalty = {'constant'};
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
    defCell = add_line(defCell, 'dim', dim, 1, 'uint', 'define');
    defCell = add_line(defCell, 'n_s', n_s, 1, 'uint', 'define');
    defCell = add_line(defCell, 'n_eq', n_eq, 1, 'uint', 'define');
    defCell = add_line(defCell, 'n_soc', n_soc, 1, 'uint', 'define');
    defCell = add_line(defCell, 'n_y', n_y, 1, 'uint', 'define');
    defCell = add_line(defCell, 'n_box', n_box, 1, 'uint', 'define');
    defCell = add_line(defCell, 'nrow_C', vars.C_CSR.nrow, 1, 'uint', 'define');
    defCell = add_line(defCell, 'nrow_Ct', vars.Ct_CSR.nrow, 1, 'uint', 'define');
    
    defCell = add_line(defCell, 'k_max', solver_options.k_max, 1, 'uint', 'define');
    defCell = add_line(defCell, 'tol_p', solver_options.tol_p, 1, precision, 'define');
    defCell = add_line(defCell, 'tol_d', solver_options.tol_d, 1, precision, 'define');
    defCell = add_line(defCell, 'in_engineering', solver_options.in_engineering, 1, 'bool', 'define');
    if solver_options.debug
        defCell = add_line(defCell, 'DEBUG', 1, 1, 'bool', 'define');
    end
    if recipe.options.time
        defCell = add_line(defCell, 'MEASURE_TIME', 1, 1, 'bool', 'define');
    end
    if strcmp(solver_options.method, 'SADMM')
        defCell = add_line(defCell, 'alpha_SADMM', solver_options.alpha, 1, precision, 'define');
        defCell = add_line(defCell, 'IS_SYMMETRIC', 1, 1, precision, 'define');
    end
    if solver_options.use_soc
        defCell = add_line(defCell, 'USE_SOC', 1, 1, precision, 'define');
    end
    
    % Constants
    constCell = [];
    constCell = add_line(constCell, 'rho', vars.rho, 1, precision, var_options_penalty);
    constCell = add_line(constCell, 'rho_i', vars.rho_i, 1, precision, var_options_penalty);
    constCell = add_line(constCell, 'A', vars.A, 1, precision, var_options);
    
    constCell = add_line(constCell, 'C_val', vars.C_CSR.val, 1, precision, var_options);
    constCell = add_line(constCell, 'C_col', vars.C_CSR.col-1, 1, 'int', var_options);
    constCell = add_line(constCell, 'C_row', vars.C_CSR.row-1, 1, 'int', var_options);
    
    constCell = add_line(constCell, 'Ct_val', vars.Ct_CSR.val, 1, precision, var_options);
    constCell = add_line(constCell, 'Ct_col', vars.Ct_CSR.col-1, 1, 'int', var_options);
    constCell = add_line(constCell, 'Ct_row', vars.Ct_CSR.row-1, 1, 'int', var_options);
    
%     constCell = add_line(constCell, 'C', vars.C, 1, precision, var_options);
    constCell = add_line(constCell, 'QQ', vars.Q, 1, precision, var_options);
    constCell = add_line(constCell, 'Te', vars.Te, 1, precision, var_options);
    constCell = add_line(constCell, 'Th', vars.Th, 1, precision, var_options);
    constCell = add_line(constCell, 'Se', vars.Se, 1, precision, var_options);
    constCell = add_line(constCell, 'Sh', vars.Sh, 1, precision, var_options);
    constCell = add_line(constCell, 'LB', vars.LB, 1, precision, var_options);
    constCell = add_line(constCell, 'UB', vars.UB, 1, precision, var_options);
    constCell = add_line(constCell, 'LBy', vars.LBy, 1, precision, var_options);
    constCell = add_line(constCell, 'UBy', vars.UBy, 1, precision, var_options);
    if solver_options.use_soc
        constCell = add_line(constCell, 'd', vars.d, 1, precision, var_options);
    end

    constCell = add_line(constCell, 'M1', vars.M1, 1, precision, var_options);
    constCell = add_line(constCell, 'M2', vars.M2, 1, precision, var_options);
           
    if solver_options.in_engineering
        constCell = add_line(constCell, 'scaling_x', vars.scaling_x, 1, precision, var_options);
        constCell = add_line(constCell, 'scaling_u', vars.scaling_u, 1, precision, var_options);
        constCell = add_line(constCell, 'scaling_i_u', vars.scaling_i_u, 1, precision, var_options);
        constCell = add_line(constCell, 'OpPoint_x', vars.OpPoint_x, 1, precision, var_options);
        constCell = add_line(constCell, 'OpPoint_u', vars.OpPoint_u, 1, precision, var_options);
    end
    
    % Variables
    varsCell = [];
%     varsCell = add_line(varsCell, 'bh', vars.bh, 1, precision);
    
    %% Declare an empty constructor object
    constructor = Spcies_constructor;
    
    %% Fill in the files
    
    % .c file
    constructor = constructor.new_empty_file('code', recipe.options, 'c');
    constructor.files.code.blocks = {'$START$', C_code.get_generic_solver_struct;...
                                     '$INSERT_SOLVER$', [this_path '/code_ellipHMPC_ADMM_C.c']};
    
    % .h file
    constructor = constructor.new_empty_file('header', recipe.options, 'h');
    constructor.files.header.blocks = {'$START$', [this_path '/header_ellipHMPC_ADMM_C.h']};
    
    % Data
    constructor.data = {'$INSERT_DEFINES$', defCell;...
                        '$INSERT_CONSTANTS$', constCell};

                        %'$INSERT_VARIABLES$', varsCell
    
end
