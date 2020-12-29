%% decVar - C version
% Writes the code for declaring a variable in Arduino
%
% INPUTS: 
%   - var: variable to be declared. Single-rowed cell with 5 columns.
%          Each column contains the following information:
%          1: name. String containing the name of the variable.
%          2: value. Variable to be declared.
%          3: initialize. Boolean that indicates if the variable is
%             initialized to the variable given in the field "value".
%          4: type. String defining the type of variable. Can be any of the following:
%                   - float, double, bool, int, uint, udint, dint, usint
%          5: class. String defining the class of variable. Can be any of the following:
%                   - define: a C #define
%                   - constant: sets the variable as a C constant
%                   - variable: a normal C variable
% 
% OUTPUTS:
%   - s: String containing the text for the variable declaration.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function s = decVar(var)

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

        if length(dim) == 2 % Vector o multidimensional array (2D or 3D)

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
        type = 'float';
    elseif strcmp(type, 'double')
        type = 'double';
    elseif strcmp(type, 'bool')
        type = 'boolean';
    elseif strcmp(type, 'int')
        type = 'int';
    elseif strcmp(type, 'uint')
        type = 'unsigned int';
    elseif strcmp(type, 'udint')
        type = 'unsigned long';
    elseif strcmp(type, 'dint')
        type = 'long';
    elseif strcmp(type, 'usint')
        type = 'unsigned long';
    else
        initialize = 0;
    end

    %% Create string for variable declaration
    
    % Add the #define tag if the class is set to 'define'
    if strcmp(class, 'define')
        s = [s sprintf('#define %s ', name) writeValue(value, type) '\n'];
    else
    
    % Add the const tag if the class is set to 'constant'
    if strcmp(class, 'constant')
        s = [s sprintf('const ')];
    end
    
    % Add the const tag if the class is set to 'constant'
    if strcmp(class, 'static constant')
        s = [s sprintf('static const ')];
    end
    
    % Set type and name
    s = [s sprintf('%s %s', type, name)];
    
    % Set dimensions
    if strcmp(order, 'vector')
        s = [s sprintf('[%d]', max(dim))];
    elseif strcmp(order, 'matrix')
        s = [s sprintf('[%d][%d]', dim(1), dim(2))];
    elseif strcmp(order, '3Dmatrix')
        s = [s sprintf('[%d][%d][%d]', dim(3), dim(1), dim(2))];
    end
    
    if initialize
        s = [s sprintf(' = ')];
        
        % Scalar
        if strcmp(order, 'scalar')
            s = [s writeValue(value, type)];

        % Vector
        elseif strcmp(order, 'vector')
            s = [s sprintf('{ ')];
            write_comma = false;
            for i=1:max(dim)
                if write_comma
                    s = [s sprintf(', ')];
                else
                    write_comma = true;
                end
                s = [s writeValue(value(i), type)];
            end
            s = [s sprintf(' }')];

        % Matrix
        elseif strcmp(order, 'matrix')
            s = [s sprintf('{ ')];
            for i = 1:dim(1)
                write_comma = false;
                s = [s sprintf('{')];
                for j = 1:dim(2)
                    if write_comma
                        s = [s sprintf(', ')];
                    else
                        write_comma = true;
                    end
                    s = [s writeValue(value(i, j), type)];   
                end
                s = [s sprintf('}')];
                if i < dim(1)
                        s = [s ', '];
                end
            end
            s = [s sprintf(' }')];

        % 3D Matrix
        elseif strcmp(order, '3Dmatrix')
            s = [s sprintf('{ ')];
            for k = 1:dim(3)
                s = [s sprintf('{')];
                for i = 1:dim(1)
                    write_comma = false;
                        s = [s sprintf('{')];
                        for j = 1:dim(2)
                            if write_comma
                                s = [s sprintf(', ')];
                            else
                                write_comma = true;
                            end
                            s = [s writeValue(value(i, j, k), type)];
                        end
                        s = [s sprintf('}')];
                        if i < dim(1)
                            s = [s ', '];
                        end
                end
                s = [s sprintf('}')];
                if k < dim(3)
                    s = [s ', '];
                end
            end
            s = [s sprintf(' }')];
        end

    end
    
    s = [s sprintf(';\n')];

    end

end

%% Auxiliary function that writes the value of the variable according to its type

function s = writeValue(value, type)
    max_inf = 1e20;
    min_inf = -1e20;
    value(value==inf) = max_inf;
    value(value==-inf) = min_inf;
    if value==-inf
        disp('here');
    end
    
    if contains(type, 'int')
        s = sprintf('%d', value);

    elseif contains(type, 'bool')
        if value == true
            s = sprintf('1');
        else
            s = sprintf('0');
        end

    else
        s = sprintf('%1.10f', value);
    end
    
end

