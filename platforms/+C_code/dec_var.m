%% dec_var - C version
%
% Writes the code for declaring a variable in C.
%
% INPUTS:
%   - var: variable to be declared. Single-rowed cell array with 6 columns.
%          Each column contains the following information:
%           - name: The name with which to declare the variable.
%           - value: A Matlab variable containing its value.
%           - initialize: Boolean that determines if it is initialized.
%           - type: This determines the type of variable. We consider:
%                   float, double, bool, int, uint, dint, udint, sint, usint
%           The following columns are optional and can be provided in any order.j
%            - define: The variable is declared as a #define.
%            - constant: The variable is declared as a constant.
%            - static: The variable is declared as static.
%            - array: Forces the variable to be an array, even if its
%                        dimension is 1.
%            - pointer: Defines the variable as a pointer.
% 
% OUTPUTS:
%   - s: String containing the text for the variable declaration.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function s = dec_var(var)

    %% Preliminaries
    s = ''; % Initialize the output to an empty string

    % Extract from the columns of var
    name = var{1};
    value = var{2};
    initialize = var{3};
    type = var{4};
    if length(var) > 4
        options = var{5};
    end
    
    % Transform type to a C type
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
    elseif strcmp(type, 'dint')
        type = 'long';
    elseif strcmp(type, 'udint')
        type = 'unsigned long';
    elseif strcmp(type, 'sint')
        type = 'int';
    elseif strcmp(type, 'usint')
        type = 'unsigned int';
    else
        error('Spcies:dec_var:type_not_recognized', 'Type of variable not recognized');
    end
    
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
    
    if strcmp(order, 'scalar') && any(strcmp(options, 'array'))
        order = 'vector';
    end

    %% Create string for variable declaration
    
    % Add the #define tag if the class is set to 'define'
    if any(strcmp(options, 'define'))
        if initialize
            if strcmp(order, 'scalar')
                s = strcat(s, sprintf('#define %s %s', name, write_value(value, type)), '\n');
            else
                warning(['Variable ' name ' cannot be initialized as #define because it is not a scalar']);
            end
        else
            s = strcat(s, sprintf('#define %s', name), '\n');
        end
    else
    
    % Add the type, pointer and name tags
    if any(strcmp(options, 'pointer'))
        pointer_tag = '*';
    else
        pointer_tag = '';
    end
    s = strcat(s, [type ' ' pointer_tag name]);
    
    % Set dimensions
    if strcmp(order, 'vector')
        s = strcat(s, sprintf('[%d]', max(dim)));
        
    elseif strcmp(order, 'matrix')
        s = strcat(s, sprintf('[%d][%d]', dim(1), dim(2)));
        
    elseif strcmp(order, '3Dmatrix')
        s = strcat(s, sprintf('[%d][%d][%d]', dim(3), dim(1), dim(2)));
        
    end
    
    if initialize
        s = strcat(s, ' = ');
        
        % Scalar
        if strcmp(order, 'scalar')
            
            s = strcat(s, write_value(value, type));
        
        % Vector
        elseif strcmp(order, 'vector')
            
            s = strcat(s, '{ ');
            
            write_comma = false;
            for i=1:max(dim)
                if write_comma
                    s = strcat(s, ', ');
                else
                    write_comma = true;
                end
                s = strcat(s, write_value(value(i), type));
            end
            
            s = strcat(s, ' }');

        % Matrix
        elseif strcmp(order, 'matrix')
            
            s = strcat(s, '{ ');
            
            for i = 1:dim(1)
                
                write_comma = false;
                s = strcat(s, '{');
                
                for j = 1:dim(2)
                    if write_comma
                        s = strcat(s, ', ');
                    else
                        write_comma = true;
                    end
                    s = strcat(s, write_value(value(i, j), type));   
                end
                
                s = strcat(s, '}');
                if i < dim(1)
                        s = strcat(s, ', ');
                end
                
            end
            
            s = strcat(s, ' }');

        % 3D Matrix
        elseif strcmp(order, '3Dmatrix')
            
            s = strcat(s, '{ ');
            
            for k = 1:dim(3)
                
                s = strcat(s, '{');
                
                for i = 1:dim(1)
                    
                    write_comma = false;
                        s = strcat(s, '{');
                        
                        for j = 1:dim(2)
                            
                            if write_comma
                                s = strcat(s, ', ');
                            else
                                write_comma = true;
                            end
                            s = strcat(s, write_value(value(i, j, k), type));
                            
                        end
                        
                        s = strcat(s, '}');
                        
                        if i < dim(1)
                            s = strcat(s, ', ');
                        end
                        
                end
                
                s = strcat(s, '}');
                
                if k < dim(3)
                    s = strcat(s, ', ');
                end
                
            end
            
            s = strcat(s, ' }');
            
        end

    end
    
    % Add the end-of-line
    s = strcat(s, ';\n');
    
     % Add the const tag
    if any(strcmp(options, 'constant'))
        s = ['const ', s];
    end
    
    % Add the static tag 
    if any(strcmp(options, 'static'))
        s = ['static ', s];
    end

    end

end

%% Auxiliary function that writes the value of the variable according to its type

function s = write_value(value, type)

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
        s = sprintf('%1.15f', value);
    end
    
end

