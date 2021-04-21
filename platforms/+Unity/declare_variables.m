%% declare_variables - Unity version
%
% Function that declares the variables for Unity Pro XL
% 
% INPUTS:
%   - vars: Cell array containing the variables to be declared.
%           Each row contains the information to declare each variable.
%           The information provided in each column can be found in the
%           documentation of +Unity/dec_var.m
%
% OUTPUTS:
%   - s: String containing the text for the variable declaration in the Unity Pro XL.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function s = declare_variables(vars)
    
    %% Preliminaries
    s = ['']; % Initialize s to an empty string
    numVars = size(vars, 1); % Number of variables stored in vars (number of rows)
    
    %% Input variables
    s = [s sprintf('<inputParameters>\n')];
    
    position = 1;
    for i = 1:numVars
        if strcmp(vars{i, 5}, 'input')
            s = [s Unity.dec_var(vars(i,:), position)];
            position = position + 1;
        end
    end
    
    s = [s sprintf('</inputParameters>\n')];
    
    %% Output variables
    s = [s '<outputParameters>\n'];
    
    position = 1;
    for i = 1:numVars
        if strcmp(vars{i, 5}, 'output')
            s = [s Unity.dec_var(vars(i,:), position)];
            position = position + 1;
        end
    end
    
    s = [s '</outputParameters>\n'];
    
    %% Public variables
    s = [s '<publicLocalVariables>\n'];
    
    for i = 1:numVars
        if strcmp(vars{i, 5}, 'public') 
            s = [s Unity.dec_var(vars(i,:))];
        end
    end
    
    s = [s '</publicLocalVariables>\n'];
    
    %% Private variables
    s = [s '<privateLocalVariables>\n'];
    
    for i = 1:numVars
        if strcmp(vars{i, 5}, 'private') 
            s = [s Unity.dec_var(vars(i,:))];
        end
    end
    
    s = [s '</privateLocalVariables>\n'];
    
end

