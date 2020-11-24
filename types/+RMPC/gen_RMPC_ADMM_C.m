%% gen_RMPC_ADMM_C - Generates the RMPC controller solved with the ADMM algorithm for C
% This version uses the projection algorithm onto the ellipsoid

% Author: Pablo Krupa (pkrupa@us.es)
% 
% Changelog: 
%   v0.1 (2020/10/16): Initial commit version
%

function gen_RMPC_ADMM_C(stru, options, save_name)
    import utils.addLine
    
    %% Rename variables for convenience
    n = stru.n;
    m = stru.m;
    N =  stru.N;
    
    %% Create vars cell matrix: Name, value, initialize, type(int, float, etc), class(variable, constant, define, etc)
    vars = cell(1, 5);
    idx = 1;
    
    % Defines
    [vars, idx] = addLine(vars, idx, 'n', n, 1, 'uint', 'define');
    [vars, idx] = addLine(vars, idx, 'm', m, 1, 'uint', 'define');
    [vars, idx] = addLine(vars, idx, 'nm', n+m, 1, 'uint', 'define');
    [vars, idx] = addLine(vars, idx, 'N', N, 1, 'uint', 'define');
    [vars, idx] = addLine(vars, idx, 'k_max', options.k_max, 1, 'uint', 'define');
    [vars, idx] = addLine(vars, idx, 'tol', options.tol, 1, 'float', 'define');
    [vars, idx] = addLine(vars, idx, 'in_engineering', options.in_engineering, 1, 'int', 'define');
    if options.debug
        [vars, idx] = addLine(vars, idx, 'debug', 1, 1, 'bool', 'define');
    end
    
    % Constants
    [vars, idx] = addLine(vars, idx, 'rho', stru.rho, 1, 'double', 'constant');
    [vars, idx] = addLine(vars, idx, 'rho_0', stru.rho_0, 1, 'double', 'constant');
    [vars, idx] = addLine(vars, idx, 'rho_N', stru.rho_N, 1, 'double', 'constant');
    [vars, idx] = addLine(vars, idx, 'rho_i', stru.rho_i, 1, 'double', 'constant');
    [vars, idx] = addLine(vars, idx, 'rho_i_0', stru.rho_i_0, 1, 'double', 'constant');
    [vars, idx] = addLine(vars, idx, 'rho_i_N', stru.rho_i_N, 1, 'double', 'constant');
    [vars, idx] = addLine(vars, idx, 'LBu0', stru.LBu0, 1, 'double', 'constant');
    [vars, idx] = addLine(vars, idx, 'UBu0', stru.UBu0, 1, 'double', 'constant');
    [vars, idx] = addLine(vars, idx, 'LBz', stru.LBz, 1, 'double', 'constant');
    [vars, idx] = addLine(vars, idx, 'UBz', stru.UBz, 1, 'double', 'constant');
    [vars, idx] = addLine(vars, idx, 'Hi', stru.Hi, 1, 'double', 'constant');
    [vars, idx] = addLine(vars, idx, 'Hi_0', stru.Hi_0, 1, 'double', 'constant');
    [vars, idx] = addLine(vars, idx, 'Hi_N', stru.Hi_N, 1, 'double', 'constant');
    [vars, idx] = addLine(vars, idx, 'AB', stru.AB, 1, 'double', 'constant');
    [vars, idx] = addLine(vars, idx, 'P', stru.P, 1, 'double', 'constant');
    [vars, idx] = addLine(vars, idx, 'P_half', stru.P_half, 1, 'double', 'constant');
    [vars, idx] = addLine(vars, idx, 'Pinv_half', stru.Pinv_half, 1, 'double', 'constant');
    [vars, idx] = addLine(vars, idx, 'Alpha', stru.Alpha, 1, 'double', 'constant');
    [vars, idx] = addLine(vars, idx, 'Beta', stru.Beta, 1, 'double', 'constant');
    if options.in_engineering
        [vars, idx] = addLine(vars, idx, 'scaling_x', stru.scaling_x, 1, 'double', 'constant');
        [vars, idx] = addLine(vars, idx, 'scaling_i_u', stru.scaling_u, 1, 'double', 'constant');
        [vars, idx] = addLine(vars, idx, 'OpPoint_x', stru.OpPoint_x, 1, 'double', 'constant');
        [vars, idx] = addLine(vars, idx, 'OpPoint_u', stru.OpPoint_u, 1, 'double', 'constant');
    end
    
    % Variables
    [vars, idx] = addLine(vars, idx, 'z', zeros(m+n, N-1), 0, 'double', 'variable');
    [vars, idx] = addLine(vars, idx, 'z_0', zeros(m, 1), 0, 'double', 'variable');
    [vars, idx] = addLine(vars, idx, 'z_N', zeros(n, 1), 0, 'double', 'variable');
    [vars, idx] = addLine(vars, idx, 'z1', zeros(m+n, N-1), 0, 'double', 'variable');
    [vars, idx] = addLine(vars, idx, 'z1_0', zeros(m, 1), 0, 'double', 'variable');
    [vars, idx] = addLine(vars, idx, 'z1_N', zeros(n, 1), 0, 'double', 'variable');
    [vars, idx] = addLine(vars, idx, 'z_hat', zeros(m+n, N-1), 0, 'double', 'variable');
    [vars, idx] = addLine(vars, idx, 'z_hat_0', zeros(m, 1), 0, 'double', 'variable');
    [vars, idx] = addLine(vars, idx, 'z_hat_N', zeros(n, 1), 0, 'double', 'variable');
    [vars, idx] = addLine(vars, idx, 'aux_N', zeros(n, 1), 0, 'double', 'variable');
    [vars, idx] = addLine(vars, idx, 'lambda', zeros(m+n, N-1), 0, 'double', 'variable');
    [vars, idx] = addLine(vars, idx, 'lambda_0', zeros(m, 1), 0, 'double', 'variable');
    [vars, idx] = addLine(vars, idx, 'lambda_N', zeros(n, 1), 0, 'double', 'variable');
    [vars, idx] = addLine(vars, idx, 'mu', zeros(n, N), 0, 'double', 'variable');
    [vars, idx] = addLine(vars, idx, 'res', zeros(n+m, N+1), 0, 'double', 'variable');
    [vars, idx] = addLine(vars, idx, 'res_1', 1, 0, 'double', 'variable');
    [vars, idx] = addLine(vars, idx, 'b', zeros(n, 1), 0, 'double', 'variable');
    [vars, idx] = addLine(vars, idx, 'project_me', zeros(n, 1), 0, 'double', 'variable');
    [vars, idx] = addLine(vars, idx, 'vPv', 0, 0, 'double', 'variable');
    
    % Inputs and outputs
    %[vars, idx] = addLine(vars, idx, 'u', zeros(m, 1), 0, 'double', 'variable');
    %[vars, idx] = addLine(vars, idx, 'x', zeros(n, 1), 0, 'double', 'variable');
    %[vars, idx] = addLine(vars, idx, 'z_star', zeros(N*(n+m), 1), 0, 'double', 'variable');
    %[vars, idx] = addLine(vars, idx, 'exit_flag', 1, 0, 'int', 'variable');
    %[vars, idx] = addLine(vars, idx, 'iter', 1, 0, 'int', 'variable');
    
    %% Create text for variables
    var_text = C_code.declareVariables(vars);
    
    %% Create text for code
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    code_text = fileread([this_path '/code_RMPC_ADMM_C.txt']);
    header_text = fileread([this_path '/header_RMPC_ADMM_C.txt']);
    
    %% Merge text
    %controller_text = [var_text '\n\n' code_text];
    controller_text = strrep(code_text, "$INSERT_NAME$", save_name);
    controller_text = strrep(controller_text, "$INSERT_VARIABLES$", var_text);
    header_text = strrep(header_text, "$INSERT_NAME$", save_name);
    
    %% Generate files for the controller
    controller_file = fopen([save_name '.c'], 'wt');
    fprintf(controller_file, controller_text);
    fclose(controller_file);
    
    header_file = fopen([save_name '.h'], 'wt');
    fprintf(header_file, header_text);
    fclose(header_file);
    
end
