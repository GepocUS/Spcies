
function constructor = cons_ellipMPC_ADMM_soc_C(recipe)

    %% Preliminaries
    import utils.add_line

    % Get path to this directory
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);

    %% Default solver options
    def_solver_options = ellipMPC.def_options_ellipMPC_ADMM_soc();
    
    % Fill recipe.solver_options with the defaults
    solver_options = utils.add_default_options_to_struct(recipe.solver_options, def_solver_options);
    recipe.solver_options = solver_options;
    
    %% Compute the ingredients of the controller
    vars = ellipMPC.compute_ellipMPC_ADMM_soc_ingredients(recipe.controller, solver_options, recipe.options);
    
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
        var_options = {'static', 'constant'};
    else
        var_options = {'constant'};
    end
    
    % Determine if float or double variables are used
    precision = recipe.options.precision;
    
    %% Create vars cell matrix: Name, value, initialize, type(int, float, etc), class(variable, constant, define, etc)
    
    % Defines
    defCell = [];
    defCell = add_line(defCell, 'dim', vars.dim, 1, 'uint', 'define');
    defCell = add_line(defCell, 'n_s', vars.n_s, 1, 'uint', 'define');
    defCell = add_line(defCell, 'n_eq', vars.n_eq, 1, 'uint', 'define');
    defCell = add_line(defCell, 'nn', n, 1, 'uint', 'define');
    defCell = add_line(defCell, 'mm', m, 1, 'uint', 'define');
    defCell = add_line(defCell, 'nm', n+m, 1, 'uint', 'define');
    defCell = add_line(defCell, 'NN', N, 1, 'uint', 'define');
    defCell = add_line(defCell, 'nrow_GhHhi', vars.GhHhi_CSR.nrow, 1, 'uint', 'define');
    defCell = add_line(defCell, 'nrow_HhiGh', vars.HhiGh_CSR.nrow, 1, 'uint', 'define');
    defCell = add_line(defCell, 'nrow_Hhi', vars.Hhi_CSR.nrow, 1, 'uint', 'define');
    defCell = add_line(defCell, 'k_max', solver_options.k_max, 1, 'uint', 'define');
    defCell = add_line(defCell, 'tol_p', solver_options.tol_p, 1, precision, 'define');
    defCell = add_line(defCell, 'tol_d', solver_options.tol_d, 1, precision, 'define');
    defCell = add_line(defCell, 'in_engineering', solver_options.in_engineering, 1, 'bool', 'define');
    if solver_options.debug
        defCell = add_line(defCell, 'DEBUG', 1, 1, 'bool', 'define');
    end
    
    % Constants
    constCell = [];
    constCell = add_line(constCell, 'rho', vars.rho, 1, precision, var_options);
    constCell = add_line(constCell, 'rho_i', vars.rho_i, 1, precision, var_options);
    constCell = add_line(constCell, 'sigma', vars.sigma, 1, precision, var_options);
    constCell = add_line(constCell, 'sigma_i', vars.sigma_i, 1, precision, var_options);
    constCell = add_line(constCell, 'Q', vars.Q, 1, precision, var_options);
    constCell = add_line(constCell, 'R', vars.R, 1, precision, var_options);
    constCell = add_line(constCell, 'T', vars.T, 1, precision, var_options);
    constCell = add_line(constCell, 'A', vars.A, 1, precision, var_options);
    constCell = add_line(constCell, 'LB', vars.LB, 1, precision, var_options);
    constCell = add_line(constCell, 'UB', vars.UB, 1, precision, var_options);
    constCell = add_line(constCell, 'PhiP', vars.PhiP, 1, precision, var_options);
    constCell = add_line(constCell, 'L_val', vars.L_CSC.val, 1, precision, var_options);
    constCell = add_line(constCell, 'L_col', vars.L_CSC.col-1, 1, 'int', var_options);
    constCell = add_line(constCell, 'L_row', vars.L_CSC.row-1, 1, 'int', var_options);
    constCell = add_line(constCell, 'Dinv', vars.Dinv, 1, precision, var_options);
    constCell = add_line(constCell, 'GhHhi_val', vars.GhHhi_CSR.val, 1, precision, var_options);
    constCell = add_line(constCell, 'GhHhi_col', vars.GhHhi_CSR.col-1, 1, 'int', var_options);
    constCell = add_line(constCell, 'GhHhi_row', vars.GhHhi_CSR.row-1, 1, 'int', var_options);
    constCell = add_line(constCell, 'HhiGh_val', vars.HhiGh_CSR.val, 1, precision, var_options);
    constCell = add_line(constCell, 'HhiGh_col', vars.HhiGh_CSR.col-1, 1, 'int', var_options);
    constCell = add_line(constCell, 'HhiGh_row', vars.HhiGh_CSR.row-1, 1, 'int', var_options);
    constCell = add_line(constCell, 'Hhi_val', vars.Hhi_CSR.val, 1, precision, var_options);
    constCell = add_line(constCell, 'Hhi_col', vars.Hhi_CSR.col-1, 1, 'int', var_options);
    constCell = add_line(constCell, 'Hhi_row', vars.Hhi_CSR.row-1, 1, 'int', var_options);
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
                                     '$INSERT_SOLVER$', [this_path '/code_ellipMPC_ADMM_soc_C.c']};
      
    % .h file
    constructor = constructor.new_empty_file('header', recipe.options, 'h');
    constructor.files.header.blocks = {'$START$', [this_path '/header_ellipMPC_ADMM_soc_C.h']};
    
    % Data
    constructor.data = {'$INSERT_DEFINES$', defCell;...
                        '$INSERT_CONSTANTS$', constCell};

end
