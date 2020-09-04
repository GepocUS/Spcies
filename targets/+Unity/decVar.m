%% decVar - Unity version
% Writes the code for declaring a variable in Unity
%
% INPUTS: 
%   - var: variable to be declared. Single-rowed cell with 5 columns.
%          Each column contains the following information:
%          1: name. String containing the name of the variable.
%          2: value. Variable to be declared.
%          3: initialize. Boolean that indicates if the variable is
%             initialized to the variable given in the field "value".
%          4: type. String defining the type of variable. Can be any of the following:
%                   - float, double, time, bool, int, uint, udint, dint, usint
%          5: class. String defining the class of variable. Can be any of the following:
%                   - input, output, public, private
% 
% OUTPUTS:
%   - s: String containing the text for the variable declaration.
%

% Function belonging to the Spcies toolbox
% Author: Pablo Krupa, pkrupa@us.es, (c) 2020
%

% Devlog and notes:
%   - 20/09/04: Cleaned code and added documentation

function s = decVar(var, position)
    if nargin < 2
        position = [];
    end
    
    %% Initialize and rename the fields of "var"
    s = [''];
    name = var{1};
    value = var{2};
    initialize = var{3};
    type = var{4};
    class = var{5};
    
    % Detect order of the variable: it can be a scalar, a vector, a matrix or a 3D matrix
    dim = size(value);
    if max(dim) == 1 % Scalar
        order = 'scalar';

    elseif max(dim) == 0 % Something non-dimensional
        order = 'other';

    else

        if length(dim) == 2 % Vector or multi-dimensional array (2D or 3D)

            if min(dim) == 1
                order = 'vector';
            else
                order = 'matrix';
            end

        elseif length(dim) == 3
            order = '3Dmatrix';

        else 
            error(['Size of variable ' name ' is not compatible']);
        end
    end
    
    % Get type
    if strcmp(type, 'float')
        type = 'REAL';
    elseif strcmp(type, 'double')
        type = 'REAL';
    elseif strcmp(type, 'time')
        type = 'TIME';
    elseif strcmp(type, 'bool')
        type = 'BOOL';
    elseif strcmp(type, 'int')
        type = 'INT';
    elseif strcmp(type, 'uint')
        type = 'INT';
    elseif strcmp(type, 'udint')
        type = 'INT';
    elseif strcmp(type, 'dint')
        type = 'INT';
    elseif strcmp(type, 'usint')
        type = 'INT';
    else
        initialize = 0;
    end
    
    %% Create string for variable declaration

    % Set name
    s = [s sprintf('\t<variables name="%s"', name)];

    % Set type and dimensions
    if strcmp(order, 'scalar')
        s = [s sprintf(' typeName="%s">\n', type)];
    elseif strcmp(order, 'vector')
        s = [s sprintf(' typeName="ARRAY[1..%d] OF %s">\n', max(dim), type)];
    elseif strcmp(order, 'matrix')
        s = [s sprintf(' typeName="ARRAY[1..%d,1..%d] OF %s">\n', dim(1), dim(2), type)];
    elseif strcmp(order, '3Dmatrix')
        s = [s sprintf(' typeName="ARRAY[1..%d,1..%d,1..%d] OF %s">\n', dim(1), dim(2), dim(3), type)];
    elseif strcmp(order, 'other')
        s = [s sprintf(' typeName="%s">\n', type)];
    end

    % Set position (if input or output)
    if strcmp(class, 'input') || strcmp(class, 'output')
        s = [s sprintf('\t\t<attribute name="PositionPin" value="%d"></attribute>\n', position)];
    end

    % Initiate
    if initialize

        % Scalar
        if strcmp(order, 'scalar')
            s = [s writeValue(value, type, 2)];

        % Vector
        elseif strcmp(order, 'vector')
            for i=1:max(dim)
                s = [s sprintf('\t\t<instanceElementDesc name="[%d]">\n', i)];
                    s = [s writeValue(value(i), type, 3)];
                s = [s sprintf('\t\t</instanceElementDesc>\n')];
            end

        % Matrix
        elseif strcmp(order, 'matrix')
            for i = 1:dim(1)
                s = [s sprintf('\t\t<instanceElementDesc name="[%d]">\n', i)];
                for j = 1:dim(2)
                    s = [s sprintf('\t\t\t<instanceElementDesc name="[%d]">\n', j)];
                        s = [s writeValue(value(i, j), type, 4)];
                    s = [s sprintf('\t\t\t</instanceElementDesc>\n')];
                end
                s = [s sprintf('\t\t</instanceElementDesc>\n')];
            end

        % 3D Matrix
        elseif strcmp(order, '3Dmatrix')
            for i = 1:dim(1)
                s = [s sprintf('\t\t<instanceElementDesc name="[%d]">\n', i)];
                for j = 1:dim(2)
                    s = [s sprintf('\t\t\t<instanceElementDesc name="[%d]">\n', j)];
                    for k = 1:dim(3)
                        s = [s sprintf('\t\t\t\t<instanceElementDesc name="[%d]">\n', k)];
                            s = [s writeValue(value(i, j, k), type, 5)];
                        s = [s sprintf('\t\t\t\t</instanceElementDesc>\n')];
                    end
                    s = [s sprintf('\t\t\t</instanceElementDesc>\n')];
                end
                s = [s sprintf('\t\t</instanceElementDesc>\n')];
            end
        end
    end
    s = [s sprintf('\t</variables>\n')];
    
end

%% Auxiliary function that writes the value of the variable according to its type
    
function s = writeValue(value, type, ntabs)
    tabs = [''];
    for i = 1:ntabs, tabs = [tabs '\t']; end
     
    if contains(type, 'INT')
        s = sprintf('%s<variableInit value="%d"></variableInit>\n', tabs, value);

    elseif contains(type, 'BOOL')
        if value == true
            s = sprintf('%s<variableInit value="TRUE"></variableInit>\n', tabs);
        else
            s = sprintf('%s<variableInit value="FALSE"></variableInit>\n', tabs);
        end

    elseif contains(type, 'TIME')
        s = sprintf('%s<variableInit value="T#%dMS"></variableInit>\n', tabs, value);

    else
        s = sprintf('%s<variableInit value="%1.18f"></variableInit>\n', tabs, value);
    end
end

