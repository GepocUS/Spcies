%% declareVariables - C version
%
% Function that declares the variables for C
% 
% INPUTS:
%   - vars: cell containing the information of each variable in each row.
%           Each row contains the following information:
%           vars = (name, value, initialize, type, class)
%           For a description of each field see the function +C/decVar.m
%           C does not require any additional steps before calling the decVar function.
%
% OUTPUTS:
%   - s: String containing the text for the variable declaration in C.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function s = declareVariables(vars)
    import C_code.decVar;

    %% Preliminaries
    s = ['']; % Initialize s to an empty string
    numVars = size(vars, 1); % Number of variables stored in vars (number of rows)

    % Generate string s
    for i = 1:numVars
        s = [s decVar(vars(i,:))];
    end

end

