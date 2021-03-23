%% gen_MPCT_extended_ss_ADMM_C
%
% Generates the C code of the MPCT solver using ADMM on an extended state space
% 
% The solver extends the state and control inputs by adding the artificial reference to them.
% Currently, there is no additional documentation available for the solver.
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

function gen_MPCT_extended_ss_ADMM_C(vars, options, spcies_options)
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
    defCell = addLine(defCell, 'dnm', 2*(n+m), 1, 'uint', 'define');
    defCell = addLine(defCell, 'nrow_AHi', vars.AHi_CSR.nrow, 1, 'uint', 'define');
    defCell = addLine(defCell, 'nrow_HiA', vars.HiA_CSR.nrow, 1, 'uint', 'define');
    defCell = addLine(defCell, 'NN', N, 1, 'uint', 'define');
    defCell = addLine(defCell, 'k_max', options.k_max, 1, 'uint', 'define');
    defCell = addLine(defCell, 'tol', options.tol, 1, 'float', 'define');
    defCell = addLine(defCell, 'in_engineering', options.in_engineering, 1, 'bool', 'define');
    if options.debug
        defCell = addLine(defCell, 'DEBUG', 1, 1, 'bool', 'define');
    end
    if vars.rho_is_scalar
        defCell = addLine(defCell, 'SCALAR_RHO', 1, 0, 'bool', 'define');
    end
    
    % Constants
    constCell = [];
    constCell = addLine(constCell, 'rho', vars.rho, 1, 'float', const_type);
    constCell = addLine(constCell, 'rho_i', vars.rho_i, 1, 'float', const_type);
    constCell = addLine(constCell, 'Tz', vars.Tz, 1, 'float', const_type);
    constCell = addLine(constCell, 'Sz', vars.Sz, 1, 'float', const_type);
    constCell = addLine(constCell, 'LB', vars.LB, 1, 'float', const_type);
    constCell = addLine(constCell, 'UB', vars.UB, 1, 'float', const_type);
    constCell = addLine(constCell, 'L_val', vars.L_CSC.val, 1, 'float', const_type);
    constCell = addLine(constCell, 'L_col', vars.L_CSC.col-1, 1, 'int', const_type);
    constCell = addLine(constCell, 'L_row', vars.L_CSC.row-1, 1, 'int', const_type);
    constCell = addLine(constCell, 'Dinv', vars.Dinv, 1, 'float', const_type);
    constCell = addLine(constCell, 'AHi_val', vars.AHi_CSR.val, 1, 'float', const_type);
    constCell = addLine(constCell, 'AHi_col', vars.AHi_CSR.col-1, 1, 'int', const_type);
    constCell = addLine(constCell, 'AHi_row', vars.AHi_CSR.row-1, 1, 'int', const_type);
    constCell = addLine(constCell, 'HiA_val', vars.HiA_CSR.val, 1, 'float', const_type);
    constCell = addLine(constCell, 'HiA_col', vars.HiA_CSR.col-1, 1, 'int', const_type);
    constCell = addLine(constCell, 'HiA_row', vars.HiA_CSR.row-1, 1, 'int', const_type);
    constCell = addLine(constCell, 'Hi_val', vars.Hi_CSR.val, 1, 'float', const_type);
    constCell = addLine(constCell, 'Hi_col', vars.Hi_CSR.col-1, 1, 'int', const_type);
    constCell = addLine(constCell, 'Hi_row', vars.Hi_CSR.row-1, 1, 'int', const_type);
    if options.in_engineering
        constCell = addLine(constCell, 'scaling_x', vars.scaling_x, 1, 'double', const_type);
        constCell = addLine(constCell, 'scaling_u', vars.scaling_u, 1, 'double', const_type);
        constCell = addLine(constCell, 'scaling_i_u', vars.scaling_i_u, 1, 'double', const_type);
        constCell = addLine(constCell, 'OpPoint_x', vars.OpPoint_x, 1, 'double', const_type);
        constCell = addLine(constCell, 'OpPoint_u', vars.OpPoint_u, 1, 'double', const_type);
    end
    
    % Declare variables
    defines_text = C_code.declareVariables(defCell);
    constants_text = C_code.declareVariables(constCell);
    variables_text = '';
    
    %% Load the different text files needed to construct the solver
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    
    controller_text = fileread([this_path '/struct_MPCT_extended_ss_ADMM_C_plain.c']);
    
    solver_text = fileread([this_path '/code_MPCT_extended_ss_ADMM_C.c']);
    solver_text = strrep(solver_text, "%", '%%');
    
    header_text = fileread([this_path '/header_MPCT_extended_ss_ADMM_C.h']);
    
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

