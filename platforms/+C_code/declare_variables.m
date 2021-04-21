%% declare_variables - C version
%
% Function that declares the variables for C
% 
% INPUTS:
%   - vars: Cell array containing the variables to be declared.
%           Each row contains the information to declare each variable.
%           The information provided in each column can be found in the
%           documentation of +C_code/dec_var.m
%
% OUTPUTS:
%   - s: String containing the text for the variable declaration in C.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function s = declare_variables(vars)

    %% Preliminaries
    s = ['']; % Initialize s to an empty string
    numVars = size(vars, 1); % Number of variables stored in vars (number of rows)

    % Generate string s
    for i = 1:numVars
        s = [s C_code.dec_var(vars(i,:))];
    end

end

