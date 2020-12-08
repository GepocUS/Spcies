%% gen_ellipMPC_ADMM_C
% Generates the text in C for the ADMM-based solver for MPC with ellipsoidal terminal constraint
% 
% Information about this formulaiton and the solver  can be found at:
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

% Author: Pablo Krupa (pkrupa@us.es)
% 
% Changelog: 
%   v0.1 (2020/10/16): Initial commit version
%   v0.2 (2020/12/07): Added parser and improved overall usability. Added documentation.
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
       if isfile([save_name '.c'])
           number = 1;
           new_save_name = [save_name '_v' num2str(number)];
           while isfile([new_save_name '.c'])
               number = number + 1;
                new_save_name = [save_name '_v' num2str(number)];
           end
           save_name = new_save_name;
       end
    end
    
    %% Rename variables for convenience
    n = vars.n;
    m = vars.m;
    N = vars.N;
    
    %% Create vars cell matrix: Name, value, initialize, type(int, float, etc), class(variable, constant, define, etc)
    varsCell = cell(1, 5);
    idx = 1;
    
    % Defines
    [varsCell, idx] = addLine(varsCell, idx, 'n', n, 1, 'uint', 'define');
    [varsCell, idx] = addLine(varsCell, idx, 'm', m, 1, 'uint', 'define');
    [varsCell, idx] = addLine(varsCell, idx, 'nm', n+m, 1, 'uint', 'define');
    [varsCell, idx] = addLine(varsCell, idx, 'N', N, 1, 'uint', 'define');
    [varsCell, idx] = addLine(varsCell, idx, 'k_max', options.k_max, 1, 'uint', 'define');
    [varsCell, idx] = addLine(varsCell, idx, 'tol', options.tol, 1, 'float', 'define');
    [varsCell, idx] = addLine(varsCell, idx, 'in_engineering', options.in_engineering, 1, 'int', 'define');
    if options.debug
        [varsCell, idx] = addLine(varsCell, idx, 'debug', 1, 1, 'bool', 'define');
    end
    
    % Constants
    [varsCell, idx] = addLine(varsCell, idx, 'rho', vars.rho, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'rho_0', vars.rho_0, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'rho_N', vars.rho_N, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'rho_i', vars.rho_i, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'rho_i_0', vars.rho_i_0, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'rho_i_N', vars.rho_i_N, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'LBu0', vars.LBu0, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'UBu0', vars.UBu0, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'LBz', vars.LBz, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'UBz', vars.UBz, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'Hi', vars.Hi, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'Hi_0', vars.Hi_0, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'Hi_N', vars.Hi_N, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'AB', vars.AB, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'P', vars.P, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'P_half', vars.P_half, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'Pinv_half', vars.Pinv_half, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'Alpha', vars.Alpha, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'Beta', vars.Beta, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'Q', vars.Q, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'R', vars.R, 1, 'double', 'constant');
    [varsCell, idx] = addLine(varsCell, idx, 'T', vars.T, 1, 'double', 'constant');
    if options.in_engineering
        [varsCell, idx] = addLine(varsCell, idx, 'scaling_x', vars.scaling_x, 1, 'double', 'constant');
        [varsCell, idx] = addLine(varsCell, idx, 'scaling_i_u', vars.scaling_u, 1, 'double', 'constant');
        [varsCell, idx] = addLine(varsCell, idx, 'OpPoint_x', vars.OpPoint_x, 1, 'double', 'constant');
        [varsCell, idx] = addLine(varsCell, idx, 'OpPoint_u', vars.OpPoint_u, 1, 'double', 'constant');
    end
    
    % Variables
    [varsCell, idx] = addLine(varsCell, idx, 'z', zeros(m+n, N-1), 0, 'double', 'variable');
    [varsCell, idx] = addLine(varsCell, idx, 'z_0', zeros(m, 1), 0, 'double', 'variable');
    [varsCell, idx] = addLine(varsCell, idx, 'z_N', zeros(n, 1), 0, 'double', 'variable');
    [varsCell, idx] = addLine(varsCell, idx, 'z1', zeros(m+n, N-1), 0, 'double', 'variable');
    [varsCell, idx] = addLine(varsCell, idx, 'z1_0', zeros(m, 1), 0, 'double', 'variable');
    [varsCell, idx] = addLine(varsCell, idx, 'z1_N', zeros(n, 1), 0, 'double', 'variable');
    [varsCell, idx] = addLine(varsCell, idx, 'aux_N', zeros(n, 1), 0, 'double', 'variable');
    [varsCell, idx] = addLine(varsCell, idx, 'mu', zeros(n, N), 0, 'double', 'variable');
    [varsCell, idx] = addLine(varsCell, idx, 'res', zeros(n+m, N+1), 0, 'double', 'variable');
    [varsCell, idx] = addLine(varsCell, idx, 'res_1', 1, 0, 'double', 'variable');
    [varsCell, idx] = addLine(varsCell, idx, 'b', zeros(n, 1), 0, 'double', 'variable');
    [varsCell, idx] = addLine(varsCell, idx, 'vPv', 0, 0, 'double', 'variable');
    [varsCell, idx] = addLine(varsCell, idx, 'c', vars.c, 1, 'double', 'variable');
    [varsCell, idx] = addLine(varsCell, idx, 'r', vars.r, 1, 'double', 'variable');
    [varsCell, idx] = addLine(varsCell, idx, 'q', zeros(n+m, 1), 0, 'double', 'variable');
    [varsCell, idx] = addLine(varsCell, idx, 'qT', zeros(n, 1), 0, 'double', 'variable');
    
    %% Create text for variables
    var_text = C_code.declareVariables(varsCell);
    
    %% Create text for code
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    code_text = fileread([this_path '/code_ellipMPC_ADMM_C.txt']);
    header_text = fileread([this_path '/header_ellipMPC_ADMM_C.txt']);
    
    %% Merge and insert text
    controller_text = strrep(code_text, "$INSERT_NAME$", save_name);
    controller_text = strrep(controller_text, "$INSERT_VARIABLES$", var_text);
    header_text = strrep(header_text, "$INSERT_NAME$", save_name);
    
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
