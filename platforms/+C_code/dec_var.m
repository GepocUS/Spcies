
function s = dec_var(var)

    %% Extract from the columns of var
    name = var{1};
    value = var{2};
    initialize = var{3};
    type = var{4};
    if length(var) > 4
        options = var{5};
    else
        options = '';
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

    % Force vector to be declared as matrix if stated splicitly
    if any(strcmp(options, 'matrix'))
        order = 'matrix'; % TODO: This still needs a better solution
    end
    
    %% Open temporary file for writing the text to
    thefile = fopen([spcies_get_root_directory '/generated_solvers/temp_write_var.txt'], 'wt');
    
    %% Write variable declaration
    if any(strcmp(options, 'define'))
        if initialize
            if strcmp(order, 'scalar')
                fprintf(thefile, sprintf('#define %s %s', name, write_value(value, type)));
            else
                warning(['Variable ' name ' cannot be initialized as #define because it is not a scalar']);
            end
        else
            fprintf(thefile, sprintf('#define %s', name));
        end
    else
        
    % Add the const tag
    if any(strcmp(options, 'constant'))
        fprintf(thefile, 'const ');
    end
    
    % Add the static tag 
    if any(strcmp(options, 'static'))
        fprintf(thefile, 'static ');
    end    
    
    % Add the type, pointer and name tags
    if any(strcmp(options, 'pointer'))
        pointer_tag = '*';
    else
        pointer_tag = '';
    end
    fprintf(thefile, [type ' ' pointer_tag name]);
    
    % Set dimensions
    if strcmp(order, 'vector')
        fprintf(thefile, sprintf('[%d]', max(dim)));
        
    elseif strcmp(order, 'matrix')
        fprintf(thefile, sprintf('[%d][%d]', dim(1), dim(2)));
        
    elseif strcmp(order, '3Dmatrix')
        fprintf(thefile, sprintf('[%d][%d][%d]', dim(3), dim(1), dim(2)));
        
    end
    
    if initialize
        fprintf(thefile, ' = ');
        
        % Scalar
        if strcmp(order, 'scalar')
            
            fprintf(thefile, write_value(value, type));
        
        % Vector
        elseif strcmp(order, 'vector')
            
            fprintf(thefile, '{ ');
            
            write_comma = false;
            for i=1:max(dim)
                if write_comma
                    fprintf(thefile, ', ');
                else
                    write_comma = true;
                end
                fprintf(thefile, write_value(value(i), type));
            end
            
            fprintf(thefile, ' }');

        % Matrix
        elseif strcmp(order, 'matrix')
            
            fprintf(thefile, '{ ');
            
            for i = 1:dim(1)
                
                write_comma = false;
                fprintf(thefile, '{');
                
                for j = 1:dim(2)
                    if write_comma
                        fprintf(thefile, ', ');
                    else
                        write_comma = true;
                    end
                    fprintf(thefile, write_value(value(i, j), type)); 
                end
                
                fprintf(thefile, '}');
                if i < dim(1)
                    fprintf(thefile, ', ');
                end
                
            end
            
            fprintf(thefile, ' }');

        % 3D Matrix
        elseif strcmp(order, '3Dmatrix')
            
            fprintf(thefile, '{ ');
            
            for k = 1:dim(3)
                
                fprintf(thefile, '{');
                
                for i = 1:dim(1)
                    
                    write_comma = false;
                    fprintf(thefile, '{');

                    for j = 1:dim(2)

                        if write_comma
                            fprintf(thefile, ', ');
                        else
                            write_comma = true;
                        end
                        fprintf(thefile, write_value(value(i, j, k), type));

                    end

                    fprintf(thefile, '}');

                    if i < dim(1)
                        fprintf(thefile, ', ');
                    end
                        
                end
                
                fprintf(thefile, '}');
                
                if k < dim(3)
                    fprintf(thefile, ', ');
                end
                
            end
            
            fprintf(thefile, ' }');
            
        end

    end
    
    % Add the end-of-line
    fprintf(thefile, ';');
    
    end
    
    fprintf(thefile, '\\n');
    
    %% Get string
    
    % Close file
    fclose(thefile);
    
    % Read the file
    s = fileread([spcies_get_root_directory '/generated_solvers/temp_write_var.txt']);
    
    % Delete file
    delete([spcies_get_root_directory '/generated_solvers/temp_write_var.txt']);
    

end

%% Auxiliary function that writes the value of the variable according to its type

function s = write_value(value, type)

    max_inf = 1e20;
    min_inf = -1e20;
    value(value==inf) = max_inf;
    value(value==-inf) = min_inf;
    
    if contains(type, 'int')
        s = sprintf('%d', value);

    elseif contains(type, 'bool')
        
        if value == true
            s = '1';
        else
            s = '0';
        end

    else
        s = sprintf('%1.15f', value);
    end
    
end
