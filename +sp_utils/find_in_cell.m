%% find_in_cell() - Returns true if the given value is contained in the given cell array
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function exists = find_in_cell(value, cell_array)

    if isa(value, 'char') || isa(value, 'string')
        indx = find(strcmp(value, cell_array));
    else
        indx = find(value == cell_array);
    end

    exists = ~isempty(indx);

end

