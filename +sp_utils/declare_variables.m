%% declare_variables
%
% Function used to obtain the variable declaration text
% of the given data for the target platform.
% 
% INPUTS:
%   - data: Cell array containing the information of the
%           variables to be declared.
%           Each row contains a variable. Each of the columns
%           contain information required to declare it.
%           Each platform may have different columns, but the 
%           first four are always the following:
%           - name: The name with which to declare the variable.
%           - value: A Matlab variable containing its value.
%           - initialize: Boolean that determines if it is initialized.
%           - type: This determines the type of variable. We consider:
%              - float: floating point number.
%              - double: double precision floating point number.
%              - bool: Boolean.
%              - int: Integer number.
%              - uint: Unsigned integer number.
%              - dint: Long integer number.
%              - udint: Unsigned long integer number.
%              - sint: Short integer number.
%              - usint: Unsigned short integer number.
%   - options: Instance of the structure Spcies_problem.options.
%
% OUTPUTS:
%   - text: Variable declaration text.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
%

function text = declare_variables(data, options)

    if strcmp(options.platform, 'C')
        text = C_code.declare_variables(data);

    elseif strcmp(options.platform, 'Arduino')
        text = Arduino.declare_variables(data);

    elseif strcmp(options.platform, 'Unity')
        text = Unity.declare_variables(data);

    end
    
    % Check if it is empty and add at least one empty string if it is
    if isempty(text)
        text{1} = "";
    end
    
end

