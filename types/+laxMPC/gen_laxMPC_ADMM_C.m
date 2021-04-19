%% gen_laxMPC_ADMM_C
%
% Generates the C code of the ADMM-based solver for the lax MPC formulation
% 
% Information about this formulation and the solver can be found at:
%
% P. Krupa, D. Limon, T. Alamo, "Implementation of model predictive control in
% programmable logic controllers", Transactions on Control Systems Technology, 2020.
% 
% Specifically, this formulation is given in equation (9) of the above reference.
% 
% INPUTS:
%   - vars: Structure containing information needed to declare the variables.
%   - options: Structure containing options of the EADMM solver.
%   - spcies_options: Structure containing the options of the toolbox.
% 
% OUTPUT: Generates the C code and saves it into the appropriate files.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function gen_laxMPC_ADMM_C(vars, options, spcies_options)
    import utils.addLine
    
    %% Defaults
    def_save_name = 'laxMPC';
    def_directory = './';
    
    %% Work with the path and save_name
    
    if isempty(spcies_options.save_name); spcies_options.save_name = default_save_name; end
    
    % Evaluate directory
    if strcmp(spcies_options.directory, '$SPCIES$'); spcies_options.directory = def_directory; end
    if ~strcmp(spcies_options.directory, '/'); spcies_options.directory = [spcies_options.directory '/']; end
    
    spcies_options.complete_path = [spcies_options.directory spcies_options.save_name];
    
    % Determine the name of the file if it already exists
    save_name = utils.process_save_name(spcies_options.complete_path, def_save_name, spcies_options.override, 'c');
    
    if options.const_are_static
        const_type = 'static constant';
    else
        const_type = 'constant';
    end
    
    %% Rename variables for convenience
    n = vars.n;
    m = vars.m;
    N = vars.N;
    
    %% Create vars cell matrix: Name, value, initialize, type(int, float, etc), class(variable, constant, define, etc)
    
    % Defines
    defCell = [];
    defCell = addLine(defCell, 'nn', n, 1, 'uint', 'define');
    defCell = addLine(defCell, 'mm', m, 1, 'uint', 'define');
    defCell = addLine(defCell, 'nm', n+m, 1, 'uint', 'define');
    defCell = addLine(defCell, 'NN', N, 1, 'uint', 'define');
    defCell = addLine(defCell, 'k_max', options.k_max, 1, 'uint', 'define');
    defCell = addLine(defCell, 'tol', options.tol, 1, 'float', 'define');
    defCell = addLine(defCell, 'in_engineering', options.in_engineering, 1, 'int', 'define');
    if options.debug
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
    if options.in_engineering
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
    
    % Declare variables
    defines_text = C_code.declareVariables(defCell);
    constants_text = C_code.declareVariables(constCell);
    variables_text = '';
    
    %% Load the different text files needed to construct the solver
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    
    controller_text = C_code.get_generic_solver_struct;
    
    solver_text = fileread([this_path '/code_laxMPC_ADMM_C.c']);
    
    header_text = fileread([this_path '/header_laxMPC_ADMM_C.h']);
    
    %% Merge and insert text
    
    % Main .c file
    controller_text = strrep(controller_text, "$INSERT_SOLVER$", solver_text); % Insert solver text
    controller_text = strrep(controller_text, "$INSERT_DEFINES$", defines_text); % Insert defines text
    controller_text = strrep(controller_text, "$INSERT_VARIABLES$", variables_text); % Insert variable text
    controller_text = strrep(controller_text, "$INSERT_CONSTANTS$", constants_text); % Insert constants text
    controller_text = strrep(controller_text, "$INSERT_NAME$", save_name); % Insert name of file
    
    % Header .h file
    header_text = strrep(header_text, "$INSERT_DEFINES$", defines_text); % Insert defines text
    header_text = strrep(header_text, "$INSERT_NAME$", save_name); % Insert name of file
    
    %% Generate files for the controller
    
    % Open write and save the controller file
    controller_file = fopen([spcies_options.directory save_name '.c'], 'wt');
    fprintf(controller_file, controller_text);
    fclose(controller_file);
    
    % Open write and save the header file
    header_file = fopen([spcies_options.directory save_name '.h'], 'wt');
    fprintf(header_file, header_text);
    fclose(header_file);
    
end
