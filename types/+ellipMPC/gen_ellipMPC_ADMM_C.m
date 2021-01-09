%% gen_ellipMPC_ADMM_C
% Generates the text in C for the ADMM-based solver for MPC with ellipsoidal terminal constraint
% 
% Information about this formulaiton and the solver can be found at:
% 
% t.b.d.
% 
% INPUTS:
%   - vars: Structure containing information needed to declare the variables.
%   - options: Structure containing several options for the solver.
%   - save_name: String containing the name of the file the controller is saved to.
%   - override: Boolean that determines is the controller is overriden if the file already exists.
% 
% OUTPUT: Saves the controller into a txt file in the current directory.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function gen_ellipMPC_ADMM_C(vars, options, save_name, override)
    import utils.addLine
    
    %% Evaluate function inputs
    def_save_name = 'ellipMPC';

    % Determine the name of the file if it already exists
    if isempty(save_name)
        save_name = def_save_name;
    end
    if ~override
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
    varsCell = cell(1, 5); idx = 1;
    [varsCell, idx] = addLine(varsCell, idx, 'nn', n, 1, 'uint', 'define');
    [varsCell, idx] = addLine(varsCell, idx, 'mm', m, 1, 'uint', 'define');
    [varsCell, idx] = addLine(varsCell, idx, 'nm', n+m, 1, 'uint', 'define');
    [varsCell, idx] = addLine(varsCell, idx, 'NN', N, 1, 'uint', 'define');
    [varsCell, idx] = addLine(varsCell, idx, 'k_max', options.k_max, 1, 'uint', 'define');
    [varsCell, idx] = addLine(varsCell, idx, 'tol', options.tol, 1, 'float', 'define');
    [varsCell, idx] = addLine(varsCell, idx, 'in_engineering', options.in_engineering, 1, 'int', 'define');
    if options.debug
        [varsCell, idx] = addLine(varsCell, idx, 'DEBUG', 1, 1, 'bool', 'define');
    end
    
    defines_text = C_code.declareVariables(varsCell);
    
    % Constants
    varsCell = cell(1, 5); idx = 1;
    [varsCell, idx] = addLine(varsCell, idx, 'rho', vars.rho, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'rho_0', vars.rho_0, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'rho_N', vars.rho_N, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'rho_i', vars.rho_i, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'rho_i_0', vars.rho_i_0, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'rho_i_N', vars.rho_i_N, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'LBu0', vars.LBu0, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'UBu0', vars.UBu0, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'LBz', vars.LBz, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'UBz', vars.UBz, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'Hi', vars.Hi, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'Hi_0', vars.Hi_0, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'Hi_N', vars.Hi_N, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'AB', vars.AB, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'P', vars.P, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'P_half', vars.P_half, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'Pinv_half', vars.Pinv_half, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'Alpha', vars.Alpha, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'Beta', vars.Beta, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'Q', vars.Q, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'R', vars.R, 1, 'double', const_type);
    [varsCell, idx] = addLine(varsCell, idx, 'T', vars.T, 1, 'double', const_type);
    if options.in_engineering
        [varsCell, idx] = addLine(varsCell, idx, 'scaling_x', vars.scaling_x, 1, 'double', const_type);
        [varsCell, idx] = addLine(varsCell, idx, 'scaling_u', vars.scaling_u, 1, 'double', const_type);
        [varsCell, idx] = addLine(varsCell, idx, 'scaling_i_u', vars.scaling_u, 1, 'double', const_type);
        [varsCell, idx] = addLine(varsCell, idx, 'OpPoint_x', vars.OpPoint_x, 1, 'double', const_type);
        [varsCell, idx] = addLine(varsCell, idx, 'OpPoint_u', vars.OpPoint_u, 1, 'double', const_type);
    end
    
    constants_text = C_code.declareVariables(varsCell);
    
    % Variables
    varsCell = cell(1, 5); idx = 1;
    [varsCell, idx] = addLine(varsCell, idx, 'c', vars.c, 1, 'double', 'variable');
    [varsCell, idx] = addLine(varsCell, idx, 'r', vars.r, 1, 'double', 'variable');
    
    variables_text = C_code.declareVariables(varsCell);
    
    %% Create text for variables
    variables_text = C_code.declareVariables(varsCell);
    
    %% Load the different text files needed to contruct the solver
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    
    controller_text = fileread([this_path '/struct_ellipMPC_ADMM_C_plain.c']);
    
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
    controller_file = fopen([save_name '.c'], 'wt');
    fprintf(controller_file, controller_text);
    fclose(controller_file);
    
    % Open write and save the header file
    header_file = fopen([save_name '.h'], 'wt');
    fprintf(header_file, header_text);
    fclose(header_file);
    
end
