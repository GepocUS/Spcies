%% decVar function for Unity

function s = decVar(var, position)
if nargin < 2
    position = [];
end

%% Preliminaries
s = [''];
name = var{1};
value = var{2};
initialize = var{3};
type = var{4};
class = var{5};

% Detect order of variable
dim = size(value);
if max(dim) == 1
    order = 'scalar';
elseif max(dim) == 0
    order = 'other';
else
    if length(dim) == 2
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
elseif strcmp(type, 'time')
    type = 'TIME';
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
    %error('decVar_Unity:VarType', ['Variable type (' type ') not recognized for variable ' name]);
    initialize = 0;
end

%% Create string for variable declaration

% If it is a deffine
if strcmp(class, 'define')
    
    s = [s sprintf('#define %s ', name) writeValue(value, type) '\n'];
  
else
    
    % Set constant status
    if strcmp(class, 'constant')
        s = [s sprintf('const ')];
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
        if strcmp(order, 'scalar')
            s = [s writeValue(value, type)];
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

function s = writeValue(value, type)
    
    if contains(type, 'int')
        s = sprintf('%d', value);
    elseif contains(type, 'Bool')
        if value == true
            s = sprintf('true');
        else
            s = sprintf('false');
        end
    elseif contains(type, 'TIME')
        s = sprintf('%d', value);
    else
        s = sprintf('%1.10f', value);
    end
    
end
