%% proj_SSOC - Projection to shifted second order cone
%
% z = proj_SSOC(x, alpha, d)
%
% This function returns the projection of vector x onto the closed convex set
%
%   K = {y = (y_0, y_1) \in \R \times \R^{n-1} : ||y_1|| <= alpha*( y_0 - d )},
%
% where alpha can be -1 or 1, and d is a scalar.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function z = proj_SSOC(x, alpha, d)
    
    % Obtain x_0 and ||x_1||
    x_0 = x(1);
    nx_1 = norm(x(2:end), 2);
    
    % Project onto K
    if nx_1 <= alpha*(x_0 - d)
        z = x;
    elseif nx_1 <= -alpha*(x_0 - d)
        z = [d; zeros(length(x) - 1, 1)];
    else
        z = 0.5*(alpha*(x_0 - d) + nx_1)*[alpha; x(2:end)./nx_1] + [d; zeros(length(x) - 1, 1)];
    end
    
end
