%% declare_variables
%
% Function used to obtain the variable declaration text
% of the given data for the target platform.
% 
% INPUTS:
%   - data: Cell array containing the information of the
%           data to be declared.
%   - options: Instance of the structure Spcies_problem.options.
%
% OUTPUTS:
%   - text: Variable declaration text.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
%

function text = declare_variables(data, options)

    if strcmp(options.platform, 'C')
        text = C_code.declareVariables(data);

    elseif strcmp(options.platform, 'Arduino')
        text = Arduino.declareVariables(data);

    elseif strcmp(options.platform, 'Unity')
        text = Unity.declareVariables(data);

    end
    
end

