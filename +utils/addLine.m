%% addLine - Function that adds a line to the dictionary that contains the variable declaration
% 
% INPUTS:
%   - dict: dictionary where the variablle is saved
%   - idx: index of the 'position' of the variable. Some embedded systems need the variables to be sorted numerically.
%   - varargin: Any and all the fields needed to define the variable. They will vary depending on the embedded system.
% 
% OUTPUTS:
%   - dict: dictionary now containing the new variable
%   - idx: index for the next variable. This is simply idx+1
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function [dict, idx] = addLine(dict, idx, varargin)
    dict(idx, :) = varargin(:);
    idx = idx + 1;
end
