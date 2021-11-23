%% proj_D - project to diamond set
%
% z = proj_D(x, lb, ub)
%
% This function returns the projection of vector x onto the closed convex set
%
% D = {y = (y_0, y_1) \in \R \times \R^{n-1} : y \in K_{-} \cap K_{+} },
%
% where K_{+} and K_{-} are given by
% 
%   K_{-} = {y = (y_0, y_1) \in \R \times \R^{n-1} : ||y_1|| <= y_0 - lb )},
%   K_{+} = {y = (y_0, y_1) \in \R \times \R^{n-1} : ||y_1|| <= ub - y_0 )},
%
% and the scalars ub and lb must satisfy ub >= lb.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function z = proj_D(x, lb, ub)

    z = utils.proj_SSOC( utils.proj_SSOC(x, 1, lb), -1, ub);
    
end
