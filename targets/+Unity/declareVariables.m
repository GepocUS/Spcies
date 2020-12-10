%% declareVariables - Unity version
% Function that declares the variables for Unity
% 
% INPUTS:
%   - vars: cell containing the information of each variable in each row.
%           Each row contains the following information:
%           vars = (name, value, initialize, type, class)
%           For a description of each field see the function +Unity/decVar.m
%           Unity distinguishes various classes of variables: inputs, outputs, public and private.
%
% OUTPUTS:
%   - s: String containing the text for the variable declaration in the Unity environment.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function s = declareVariables(vars)
    import Unity.decVar;
    
    %% Preliminaries
    s = ['']; % Initialize s to an empty string
    numVars = size(vars, 1); % Number of variables stored in vars (number of rows)
    
    %% Input variables
    s = [s sprintf('<inputParameters>\n')];
    
    position = 1;
    for i = 1:numVars
        if strcmp(vars{i, 5}, 'input')
            s = [s decVar(vars(i,:), position)];
            position = position + 1;
        end
    end
    
    s = [s sprintf('</inputParameters>\n')];
    
    %% Output variables
    s = [s '<outputParameters>\n'];
    
    position = 1;
    for i = 1:numVars
        if strcmp(vars{i, 5}, 'output')
            s = [s decVar(vars(i,:), position)];
            position = position + 1;
        end
    end
    
    s = [s '</outputParameters>\n'];
    
    %% Public variables
    s = [s '<publicLocalVariables>\n'];
    
    for i = 1:numVars
        if strcmp(vars{i, 5}, 'public') 
            s = [s decVar(vars(i,:))];
        end
    end
    
    s = [s '</publicLocalVariables>\n'];
    
    %% Private variables
    s = [s '<privateLocalVariables>\n'];
    
    for i = 1:numVars
        if strcmp(vars{i, 5}, 'private') 
            s = [s decVar(vars(i,:))];
        end
    end
    
    s = [s '</privateLocalVariables>\n'];
    
end

