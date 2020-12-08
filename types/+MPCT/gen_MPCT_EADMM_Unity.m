%% gen_MPCT_EADMM_Arduino - Generates the MPCT controller solved with the EADMM algorithm for Unity
%
% INPUTS:
%   - vars: Structure containing information needed to declare the variables
%   - options: Structure containing several options for the solver
%   - save_name: String containing the name of the file the controller is saved to
%   - override: Boolean that determines is the controller is overriden if the file already exists.
% 
%
% OUTPUT: Saves the controller into a txt file in the current directory
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

% Author: Pablo Krupa (pkrupa@us.es)
% 
% Changelog: 
%   v0.1 (2020/09/04): Initial commit version
%   v0.2 (2020/12/08): Added parser and improved overall usability
%

function gen_MPCT_EADMM_Unity(vars, options, save_name, override)
    import utils.addLine
    
    %% Evaluate function inputs
    def_save_name = 'MPCT';

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
    N =  vars.N;
    
    %% Create vars cell matrix: Name, value, initialize, type(int, float, etc), class(variable, constant, define, etc)
    varsCell = cell(1, 5);
    idx = 1;
    
    % Inputs
    [varsCell, idx] = addLine(varsCell, idx, 'ref', zeros(m+n, 1), 0, 'float', 'input');
    [varsCell, idx] = addLine(varsCell, idx, 'x0', zeros(m+n, 1), 0, 'float', 'input');
    [varsCell, idx] = addLine(varsCell, idx, 'NewST', 0, 0, 'bool', 'input');
    [varsCell, idx] = addLine(varsCell, idx, 'Manual', 0, 0, 'bool', 'input');
    [varsCell, idx] = addLine(varsCell, idx, 'u_m', zeros(m, 1), 0, 'float', 'input');
    
    % Outputs
    [varsCell, idx] = addLine(varsCell, idx, 'u', zeros(m,1), 0, 'float', 'output');
    [varsCell, idx] = addLine(varsCell, idx, 'e_flag', 0, 0, 'int', 'output');
    [varsCell, idx] = addLine(varsCell, idx, 'MPCT_done', 0, 0, 'bool', 'output');
    [varsCell, idx] = addLine(varsCell, idx, 'k', 0, 0, 'int', 'output');
    
    % Defines
    [varsCell, idx] = addLine(varsCell, idx, 'n', n, 1, 'uint', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'm', m, 1, 'uint', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'nm', n+m, 1, 'uint', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'NN', N, 1, 'uint', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'k_max', options.k_max, 1, 'uint', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'k_inc_max', options.k_max_inc, 1, 'uint', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'k_inc', 0, 0, 'int', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'tol', options.tol, 1, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'in_engineering', options.in_engineering, 1, 'bool', 'public');
    
    % Constants
    [varsCell, idx] = addLine(varsCell, idx, 'rho', vars.rho, 1, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'rho_0', vars.rho_0, 1, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'rho_s', vars.rho_s, 1, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'LB', vars.LB, 1, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'UB', vars.UB, 1, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'LB0', vars.LB0, 1, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'UB0', vars.UB0, 1, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'LBs', vars.LBs, 1, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'UBs', vars.UBs, 1, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'AB', vars.AB, 1, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'T', vars.T, 1, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'S', vars.S, 1, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'scaling', vars.scaling, 1, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'scaling_inv_u', vars.scaling_inv_u, 1, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'OpPoint', vars.OpPoint, 1, 'float', 'public');
    
    % Alpha and Beta
    [varsCell, idx] = addLine(varsCell, idx, 'Alpha', vars.Alpha, 1, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'Beta', vars.Beta, 1, 'float', 'public');
    
    % Counters
    [varsCell, idx] = addLine(varsCell, idx, 'i', 0, 1, 'int', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'j', 0, 1, 'int', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'l', 0, 1, 'int', 'public');
    
    % Variables
    [varsCell, idx] = addLine(varsCell, idx, 'z1', zeros(m+n, N+1), 0, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'z2', zeros(m+n, 1), 0, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'z3', zeros(m+n, N+1), 0, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'lambda', zeros(m+n, N+3), 0, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'q2', zeros(m+n, 1), 0, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'q3', zeros(m+n, N+1), 0, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'res', zeros(m+n, N+3), 0, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'mu3', zeros(n, N), 0, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'res_1', 0, 0, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'done', 0, 1, 'bool', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'ref_inc', zeros(m+n, 1), 0, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'x0_inc', zeros(m+n, 1), 0, 'float', 'public');
    [varsCell, idx] = addLine(varsCell, idx, 'Reset_me', 0, 0, 'bool', 'public');
    
    % QP1
    [varsCell, idx] = addLine(varsCell, idx, 'H1i', vars.H1i, 1, 'float', 'public');
    
    % QP2
    [varsCell, idx] = addLine(varsCell, idx, 'W2', vars.W2, 1, 'float', 'public');
    
    % QP3
    [varsCell, idx] = addLine(varsCell, idx, 'H3i', vars.H3i, 1, 'float', 'public');
    
    %% Create text for variables
    var_text = Unity.declareVariables(varsCell);
    
    %% Create text for code
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    code_text = fileread([this_path '/MPCT_EADMM_Unity.txt']);
    
    %% Merge text
    controller_text = Unity.merger_Unity(var_text, code_text, 'MPCT');
    
    %% Generate files for the controller
    controller_file = fopen([save_name '.txt'], 'wt');
    fprintf(controller_file, controller_text);
    fclose(controller_file);
    movefile([save_name '.txt'], [save_name '.XDB']);
    
end

