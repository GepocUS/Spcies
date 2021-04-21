%% declare_variables - Arduino version
%
% Function that declares the variables for Arduino
% 
% INPUTS:
%   - vars: Cell array containing the variables to be declared.
%           Each row contains the information to declare each variable.
%           The information provided in each column can be found in the
%           documentation of +Arduino/dec_var.m
% 
% OUTPUTS:
%   - s: String containing the text for the variable declaration in the Arduino environment.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function s = declare_variables(vars)

    %% Preliminaries
    s = ['']; % Initialize s to an empty string
    numVars = size(vars, 1); % Number of variables stored in vars (number of rows)

    % Generate string s
    for i = 1:numVars
        s = [s Arduino.dec_var(vars(i,:))];
    end

end

