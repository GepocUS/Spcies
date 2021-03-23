%% addLine - Function that adds a line to the dictionary that contains the variable declaration
% 
% INPUTS:
%   - dict: dictionary where the variable is saved
%   - varargin: Any and all the fields needed to define the variable. They will vary depending on the embedded system.
% 
% OUTPUTS:
%   - dict: dictionary now containing the new variable
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function dict = addLine(dict, varargin)
    if isempty(dict)
        dict = varargin(:)';
    else
        idx = size(dict, 1) + 1;
        dict(idx, :) = varargin(:);
    end
end
