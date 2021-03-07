%% gen_MMPCT_EADMM_C
% Generates the text in C for the EADMM-based solver for the MPCT formulation
% 
% Information about this formulation and the solver can be found at:
% 
% P. Krupa, I. Alvarado, D. Limon, T. Alamo, "Implementation of model predictive control for 
% tracking in embedded systems using a sparse extended ADMM algorithm", arXiv: 2008.09071v1, 2020.
% 
% INPUTS:
%   - vars: Structure containing information needed to declare the variables.
%   - options: Structure containing several options for the solver.
%   - spcies_options: Structure containing several options for the code generation.
% 
% OUTPUT: Saves the controller into a file in the current directory.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function gen_MPCT_EADMM_C(vars, options, spcies_options)
    import utils.addLine
    
    %% Evaluate function inputs
    def_save_name = 'MPCT_solver';

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
    defCell = addLine(defCell, 'nm', n+m, 1, 'uint', 'define');
    defCell = addLine(defCell, 'NN', N, 1, 'uint', 'define');
    defCell = addLine(defCell, 'k_max', options.k_max, 1, 'uint', 'define');
    defCell = addLine(defCell, 'tol', options.tol, 1, 'float', 'define');
    defCell = addLine(defCell, 'in_engineering', options.in_engineering, 1, 'bool', 'define');
    if options.debug
        defCell = addLine(defCell, 'DEBUG', 1, 1, 'bool', 'define');
    end
    
    % Constants
    constCell = [];
    constCell = addLine(constCell, 'rho', vars.rho, 1, 'float', const_type);
    constCell = addLine(constCell, 'rho_0', vars.rho_0, 1, 'float', const_type);
    constCell = addLine(constCell, 'rho_s', vars.rho_s, 1, 'float', const_type);
    constCell = addLine(constCell, 'LB', vars.LB, 1, 'float', const_type);
    constCell = addLine(constCell, 'UB', vars.UB, 1, 'float', const_type);
    constCell = addLine(constCell, 'LB_0', vars.LB0, 1, 'float', const_type);
    constCell = addLine(constCell, 'UB_0', vars.UB0, 1, 'float', const_type);
    constCell = addLine(constCell, 'LB_s', vars.LBs, 1, 'float', const_type);
    constCell = addLine(constCell, 'UB_s', vars.UBs, 1, 'float', const_type);
    constCell = addLine(constCell, 'AB', vars.AB, 1, 'float', const_type);
    constCell = addLine(constCell, 'T', vars.T, 1, 'float', const_type);
    constCell = addLine(constCell, 'S', vars.S, 1, 'float', const_type);
    constCell = addLine(constCell, 'Alpha', vars.Alpha, 1, 'float', const_type);
    constCell = addLine(constCell, 'Beta', vars.Beta, 1, 'float', const_type);
    constCell = addLine(constCell, 'H1i', vars.H1i, 1, 'float', const_type);
    constCell = addLine(constCell, 'W2', vars.W2, 1, 'float', const_type);
    constCell = addLine(constCell, 'H3i', vars.H3i, 1, 'float', const_type);
    if options.in_engineering
        constCell = addLine(constCell, 'scaling_x', vars.scaling_x, 1, 'double', const_type);
        constCell = addLine(constCell, 'scaling_u', vars.scaling_u, 1, 'double', const_type);
        constCell = addLine(constCell, 'scaling_i_u', vars.scaling_i_u, 1, 'double', const_type);
        constCell = addLine(constCell, 'OpPoint_x', vars.OpPoint_x, 1, 'double', const_type);
        constCell = addLine(constCell, 'OpPoint_u', vars.OpPoint_u, 1, 'double', const_type);
    end
    
    % Declare variales
    defines_text = C_code.declareVariables(defCell);
    constants_text = C_code.declareVariables(constCell);
    variables_text = '';
    
    %% Load the different text files needed to contruct the solver
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    
    controller_text = fileread([this_path '/struct_MPCT_EADMM_C_plain.c']);
    
    solver_text = fileread([this_path '/code_MPCT_EADMM_C.c']);
    
    header_text = fileread([this_path '/header_MPCT_EADMM_C.h']);
    
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

    
    
    
    
