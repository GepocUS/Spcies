%% proj_SOC - Projection to second order cone
%
% z = proj_SOC(x)
%
% Returns the projection z of the vector x onto the second order cone
%
%   { y = (y_0, y_1) \in \R \times \R^{n-1} : ||y_1|| <= y_0 }.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function z = proj_SOC(x)

    % Obtain x_0 and ||x_1||
    x_0 = x(1);
    nx_1 = norm(x(2:end), 2);
    
    % Project onto the SOC
    if nx_1 <= x_0
        z = x;
    elseif nx_1 <= -x_0
        z = zeros(length(x), 1);
    else
        z = 0.5*(x_0+ nx_1)*[1; x(2:end)./nx_1];
    end
    
end
