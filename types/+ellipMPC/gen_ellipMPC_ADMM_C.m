%% gen_ellipMPC_ADMM_C
%
% Generates the C code of the ADMM-based solver for MPC with ellipsoidal terminal constraint
% 
% Information about this formulation and the solver can be found at:
% 
% t.b.d. (it will be available shortly in an arXic preprint)
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

function gen_ellipMPC_ADMM_C(vars, options, spcies_options)
    import utils.addLine
    
    %% Evaluate function inputs
    def_save_name = 'ellipMPC';

    % Determine the name of the file if it already exists
    if isempty(spcies_options.save_name)
        save_name = def_save_name;
    else
        save_name = spcies_options.save_name;
    end
    
    if ~spcies_options.override
        save_name = utils.find_unused_file_name(save_name, 'c');
    end
    
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
    defCell= addLine(defCell, 'nm', n+m, 1, 'uint', 'define');
    defCell = addLine(defCell, 'NN', N, 1, 'uint', 'define');
    defCell = addLine(defCell, 'k_max', options.k_max, 1, 'uint', 'define');
    defCell= addLine(defCell, 'tol', options.tol, 1, 'float', 'define');
    defCell = addLine(defCell, 'in_engineering', options.in_engineering, 1, 'int', 'define');
    if options.debug
        defCell = addLine(defCell, 'DEBUG', 1, 1, 'bool', 'define');
    end
    
    % Constants
    constCell = [];
    constCell = addLine(constCell, 'LBu0', vars.LBu0, 1, 'double', const_type);
    constCell = addLine(constCell, 'UBu0', vars.UBu0, 1, 'double', const_type);
    constCell = addLine(constCell, 'LBz', vars.LBz, 1, 'double', const_type);
    constCell = addLine(constCell, 'UBz', vars.UBz, 1, 'double', const_type);
    constCell = addLine(constCell, 'Hi', vars.Hi, 1, 'double', const_type);
    constCell = addLine(constCell, 'Hi_0', vars.Hi_0, 1, 'double', const_type);
    constCell = addLine(constCell, 'Hi_N', vars.Hi_N, 1, 'double', const_type);
    constCell = addLine(constCell, 'AB', vars.AB, 1, 'double', const_type);
    constCell = addLine(constCell, 'P', vars.P, 1, 'double', const_type);
    constCell = addLine(constCell, 'P_half', vars.P_half, 1, 'double', const_type);
    constCell = addLine(constCell, 'Pinv_half', vars.Pinv_half, 1, 'double', const_type);
    constCell = addLine(constCell, 'Alpha', vars.Alpha, 1, 'double', const_type);
    constCell = addLine(constCell, 'Beta', vars.Beta, 1, 'double', const_type);
    constCell= addLine(constCell, 'Q', vars.Q, 1, 'double', const_type);
    constCell = addLine(constCell, 'R', vars.R, 1, 'double', const_type);
    constCell = addLine(constCell, 'T', vars.T, 1, 'double', const_type);
    if options.in_engineering
        constCell = addLine(constCell, idx, 'scaling_x', vars.scaling_x, 1, 'double', const_type);
        constCell = addLine(constCell, idx, 'scaling_u', vars.scaling_u, 1, 'double', const_type);
        constCell = addLine(constCell, idx, 'scaling_i_u', vars.scaling_i_u, 1, 'double', const_type);
        constCell = addLine(constCell, idx, 'OpPoint_x', vars.OpPoint_x, 1, 'double', const_type);
        constCell = addLine(constCell, idx, 'OpPoint_u', vars.OpPoint_u, 1, 'double', const_type);
    end
    
    % Variables
    varsCell = [];
    varsCell = addLine(varsCell, 'c', vars.c, 1, 'double', 'variable');
    varsCell = addLine(varsCell, 'r', vars.r, 1, 'double', 'variable');
    
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
    variables_text = C_code.declareVariables(varsCell);
    
    %% Load the different text files needed to construct the solver
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    
    controller_text = C_code.get_generic_solver_struct;
    
    solver_text = fileread([this_path '/code_ellipMPC_ADMM_C.c']);
    
    header_text = fileread([this_path '/header_ellipMPC_ADMM_C.h']);
       
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

