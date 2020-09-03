function [dict, idx] = addLine(dict, idx, varargin)
    dict(idx, :) = varargin(:);
    idx = idx + 1;
end