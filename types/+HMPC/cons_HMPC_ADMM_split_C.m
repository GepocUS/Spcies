%% cons_HMPC_ADMM_split_C
%
% Generates the constructor for C of the HMPC formulation based on ADMM which splits
% the decision variables into (z, s) and (z_hat, s_hat).
% 
% Information about this formulation can be found at:
%
% P. Krupa, D. Limon, and T. Alamo, “Harmonic based model predictive
% control for set-point tracking", IEEE Transactions on Automatic Control.
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
%                         Default values provided in def_options_HMPC_ADMM.m
%              - .sigma: Penalty parameter sigma. Scalar.
%              - .rho: Penalty parameter rho. Scalar.
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

function constructor = cons_HMPC_ADMM_split_C(recipe)
    %% Preliminaries
    import utils.add_line

    % Get path to this directory
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);

    %% Default solver options
    if strcmp(recipe.options.method, 'SADMM_split')
        def_solver_options = HMPC.def_options_HMPC_SADMM();
    else
        def_solver_options = HMPC.def_options_HMPC_ADMM();
    end
    
    % Fill recipe.solver_options with the defaults
    solver_options = utils.add_default_options_to_struct(recipe.solver_options, def_solver_options);
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
    if strcmp(recipe.options.method, 'SADMM_split')
        vars = HMPC.compute_HMPC_SADMM_split_ingredients(recipe.controller, solver_options, recipe.options);
    else
        vars = HMPC.compute_HMPC_ADMM_split_ingredients(recipe.controller, solver_options, recipe.options);
    end
    
    %% Set save_name to type if none is provided
    if isempty(recipe.options.save_name)
        recipe.options.save_name = recipe.options.type;
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
    defCell = add_line(defCell, 'nn', n, 1, 'uint', 'define');
    defCell = add_line(defCell, 'mm', m, 1, 'uint', 'define');
    defCell = add_line(defCell, 'nm', n+m, 1, 'uint', 'define');
    if ~recipe.solver_options.box_constraints
        defCell = add_line(defCell, 'n_y', n_y, 1, 'uint', 'define');
    end
    defCell = add_line(defCell, 'NN', N, 1, 'uint', 'define');
    defCell = add_line(defCell, 'dim', dim, 1, 'uint', 'define');
    defCell = add_line(defCell, 'n_s', n_s, 1, 'uint', 'define');
    defCell = add_line(defCell, 'n_eq', n_eq, 1, 'uint', 'define');
    defCell = add_line(defCell, 'n_soc', n_soc, 1, 'uint', 'define');
    if solver_options.sparse
        defCell = add_line(defCell, 'nrow_M', dim+n_eq+2*n_s, 1, 'uint', 'define');
    else
        defCell = add_line(defCell, 'NON_SPARSE', 1, 1, 'bool', 'define');
    end
    if ~solver_options.box_constraints
        defCell = add_line(defCell, 'COUPLED_CONSTRAINTS', 1, 1, 'bool', 'define');
    end
    defCell = add_line(defCell, 'k_max', solver_options.k_max, 1, 'uint', 'define');
    defCell = add_line(defCell, 'tol_p', solver_options.tol_p, 1, precision, 'define');
    defCell = add_line(defCell, 'tol_d', solver_options.tol_d, 1, precision, 'define');
    defCell = add_line(defCell, 'in_engineering', solver_options.in_engineering, 1, 'bool', 'define');
    if solver_options.debug
        defCell = add_line(defCell, 'DEBUG', 1, 1, 'bool', 'define');
    end
    if strcmp(recipe.options.method, 'SADMM_split')
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
    constCell = add_line(constCell, 'sigma', vars.sigma, 1, precision, var_options_penalty);
    constCell = add_line(constCell, 'sigma_i', vars.sigma_i, 1, precision, var_options_penalty);
    constCell = add_line(constCell, 'A', vars.A, 1, precision, var_options);
    constCell = add_line(constCell, 'Te', vars.Te, 1, precision, var_options);
    constCell = add_line(constCell, 'Se', vars.Se, 1, precision, var_options);
    constCell = add_line(constCell, 'LB', vars.LB, 1, precision, var_options);
    constCell = add_line(constCell, 'UB', vars.UB, 1, precision, var_options);
    constCell = add_line(constCell, 'LBy', vars.LBy, 1, precision, var_options);
    constCell = add_line(constCell, 'UBy', vars.UBy, 1, precision, var_options);
    if solver_options.sparse
        constCell = add_line(constCell, 'L_val', vars.L_CSC.val, 1, precision, var_options);
        constCell = add_line(constCell, 'L_col', vars.L_CSC.col-1, 1, 'int', var_options);
        constCell = add_line(constCell, 'L_row', vars.L_CSC.row-1, 1, 'int', var_options);
        constCell = add_line(constCell, 'Dinv', vars.Dinv, 1, precision, var_options);
        constCell = add_line(constCell, 'idx_x0', vars.idx_x0-1-dim-n_s, 1, 'int', var_options);
    else
        constCell = add_line(constCell, 'M1', vars.M1, 1, precision, var_options);
        if solver_options.use_soc
            constCell = add_line(constCell, 'M2', vars.M2, 1, precision, var_options);
            defCell = add_line(defCell, 'dim_M2', n_eq+n_s, 1, precision, 'define');
        else
            constCell = add_line(constCell, 'M2', vars.M2(:,1:n), 1, precision, var_options);
            defCell = add_line(defCell, 'dim_M2', n, 1, precision, 'define');
        end
    end
    if solver_options.in_engineering
        constCell = add_line(constCell, 'scaling_x', vars.scaling_x, 1, precision, var_options);
        constCell = add_line(constCell, 'scaling_u', vars.scaling_u, 1, precision, var_options);
        constCell = add_line(constCell, 'scaling_i_u', vars.scaling_i_u, 1, precision, var_options);
        constCell = add_line(constCell, 'OpPoint_x', vars.OpPoint_x, 1, precision, var_options);
        constCell = add_line(constCell, 'OpPoint_u', vars.OpPoint_u, 1, precision, var_options);
    end
    
    % Variables
    varsCell = [];
    varsCell = add_line(varsCell, 'bh', vars.bh, 1, precision);
    
    %% Declare an empty constructor object
    constructor = Spcies_constructor;
    
    %% Fill in the files
    
    % .c file
    constructor = constructor.new_empty_file('code', recipe.options, 'c');
    constructor.files.code.blocks = {'$START$', C_code.get_generic_solver_struct;...
                                     '$INSERT_SOLVER$', [this_path '/code_HMPC_ADMM_split_C.c']};
    
    % .h file
    constructor = constructor.new_empty_file('header', recipe.options, 'h');
    constructor.files.header.blocks = {'$START$', [this_path '/header_HMPC_ADMM_split_C.h']};
    
    % Data
    constructor.data = {'$INSERT_DEFINES$', defCell;...
                        '$INSERT_CONSTANTS$', constCell;...
                        '$INSERT_VARIABLES$', varsCell};

end
