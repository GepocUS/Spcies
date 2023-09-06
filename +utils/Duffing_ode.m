%% Duffing_ode - Ordinary differential equations of the Duffing oscillator
% 
% These equations correspond to the Duffing oscillator system 
% described in https://en.wikipedia.org/wiki/Duffing_equation, 
% but taking a control action u(t) instead of the cos(w*t) term.
% 
% INPUTS:
% - t: Current time (unused)
% - x: Current system state
% - u: Control action
% - param: Structure containing the alpha, beta, delta and gamma parameters
% 
% OUTPUTS:
% - xd: Time-derivative of the system state
% 

function xd = Duffing_ode(t, x, u, param)
    xd = [-param.delta*x(1) - param.alpha*x(2) - param.beta*x(2)^3 + param.gamma*u; x(1)];
end

