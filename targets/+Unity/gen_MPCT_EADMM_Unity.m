%% Funciton that creates the MPCT controller solved with the EADMM algorithm for Arduiuno

function gen_MPCT_EADMM_Unity(str, options, save_name)
import utils.addLine

%% Rename for convenience
n = str.n;
m = str.m;
N =  str.N;

%% Create vars cell matrix: Name, value, initialize, type(int, float, etc), class(variable, constant, define, etc)
vars = cell(1, 5);
idx = 1;

% Inputs
[vars, idx] = addLine(vars, idx, 'ref', zeros(m+n, 1), 0, 'float', 'input');
[vars, idx] = addLine(vars, idx, 'x0', zeros(m+n, 1), 0, 'float', 'input');
[vars, idx] = addLine(vars, idx, 'NewST', 0, 0, 'bool', 'input');
[vars, idx] = addLine(vars, idx, 'Manual', 0, 0, 'bool', 'input');
[vars, idx] = addLine(vars, idx, 'u_m', zeros(m, 1), 0, 'float', 'input');

% Outputs
[vars, idx] = addLine(vars, idx, 'u', zeros(m,1), 0, 'float', 'output');
[vars, idx] = addLine(vars, idx, 'e_flag', 0, 0, 'int', 'output');
[vars, idx] = addLine(vars, idx, 'MPCT_done', 0, 0, 'bool', 'output');
[vars, idx] = addLine(vars, idx, 'k', 0, 0, 'int', 'output');

% Defines
[vars, idx] = addLine(vars, idx, 'n', n, 1, 'uint', 'public');
[vars, idx] = addLine(vars, idx, 'm', m, 1, 'uint', 'public');
[vars, idx] = addLine(vars, idx, 'nm', n+m, 1, 'uint', 'public');
[vars, idx] = addLine(vars, idx, 'NN', N, 1, 'uint', 'public');
[vars, idx] = addLine(vars, idx, 'k_max', options.k_max, 1, 'uint', 'public');
[vars, idx] = addLine(vars, idx, 'k_inc_max', options.k_max_inc, 1, 'uint', 'public');
[vars, idx] = addLine(vars, idx, 'k_inc', 0, 0, 'int', 'public');
[vars, idx] = addLine(vars, idx, 'tol', options.tol, 1, 'float', 'public');
[vars, idx] = addLine(vars, idx, 'in_engineering', options.in_engineering, 1, 'bool', 'public');

% Constants
[vars, idx] = addLine(vars, idx, 'rho', str.rho, 1, 'float', 'public');
[vars, idx] = addLine(vars, idx, 'LB', str.LB, 1, 'float', 'public');
[vars, idx] = addLine(vars, idx, 'UB', str.UB, 1, 'float', 'public');
[vars, idx] = addLine(vars, idx, 'AB', str.AB, 1, 'float', 'public');
[vars, idx] = addLine(vars, idx, 'T', str.T, 1, 'float', 'public');
[vars, idx] = addLine(vars, idx, 'S', str.S, 1, 'float', 'public');
[vars, idx] = addLine(vars, idx, 'scaling', str.scaling, 1, 'float', 'public');
[vars, idx] = addLine(vars, idx, 'scaling_inv_u', str.scaling_inv_u, 1, 'float', 'public');
[vars, idx] = addLine(vars, idx, 'OpPoint', str.OpPoint, 1, 'float', 'public');

% Alpha and Beta
[vars, idx] = addLine(vars, idx, 'Alpha', str.Alpha, 1, 'float', 'public');
[vars, idx] = addLine(vars, idx, 'Beta', str.Beta, 1, 'float', 'public');

% Counters
[vars, idx] = addLine(vars, idx, 'i', 0, 1, 'int', 'public');
[vars, idx] = addLine(vars, idx, 'j', 0, 1, 'int', 'public');

[vars, idx] = addLine(vars, idx, 'l', 0, 1, 'int', 'public');

% Variables
[vars, idx] = addLine(vars, idx, 'z1', zeros(m+n, N+1), 0, 'float', 'public');
[vars, idx] = addLine(vars, idx, 'z2', zeros(m+n, 1), 0, 'float', 'public');
[vars, idx] = addLine(vars, idx, 'z3', zeros(m+n, N+1), 0, 'float', 'public');
[vars, idx] = addLine(vars, idx, 'lambda', zeros(m+n, N+3), 0, 'float', 'public');
% [vars, idx] = addLine(vars, idx, 'z1', z1, 1, 'float', 'variable'); % FIXME: Only for debgging
% [vars, idx] = addLine(vars, idx, 'z2', z2, 1, 'float', 'variable');
% [vars, idx] = addLine(vars, idx, 'z3', z3, 1, 'float', 'variable');
% [vars, idx] = addLine(vars, idx, 'lambda', lambda, 1, 'float', 'variable');
[vars, idx] = addLine(vars, idx, 'q2', zeros(m+n, 1), 0, 'float', 'public');
[vars, idx] = addLine(vars, idx, 'q3', zeros(m+n, N+1), 0, 'float', 'public');
[vars, idx] = addLine(vars, idx, 'res', zeros(m+n, N+3), 0, 'float', 'public');
[vars, idx] = addLine(vars, idx, 'mu3', zeros(n, N), 0, 'float', 'public');
[vars, idx] = addLine(vars, idx, 'res_1', 0, 0, 'float', 'public');
[vars, idx] = addLine(vars, idx, 'done', 0, 1, 'bool', 'public');
[vars, idx] = addLine(vars, idx, 'ref_inc', zeros(m+n, 1), 0, 'float', 'public');
[vars, idx] = addLine(vars, idx, 'x0_inc', zeros(m+n, 1), 0, 'float', 'public');
[vars, idx] = addLine(vars, idx, 'Reset_me', 0, 0, 'bool', 'public');

% QP1
[vars, idx] = addLine(vars, idx, 'H1i', str.H1i, 1, 'float', 'public');

% QP2
[vars, idx] = addLine(vars, idx, 'W2', str.W2, 1, 'float', 'public');

% QP3
[vars, idx] = addLine(vars, idx, 'H3i', str.H3i, 1, 'float', 'public');

%% Create text for variables
var_text = Unity.declareVariables(vars);

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
