%% add_line
%
% Adds a new row to the given cell array
% 
% INPUTS:
%   - dict: Cell array (or empty array) to which to add a new line
%   - varargin: Columns of the cell array to be added to the new row.
% 
% OUTPUTS:
%   - dict: Cell array including the new row
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function dict = addLine(dict, varargin)

    if isempty(dict)
        dict = varargin(:)'; % If an empty object is given then we create a new cell array
    else
        idx = size(dict, 1) + 1;
        dict(idx, :) = varargin(:); % Add the arguments to each column of the new row
    end

end

