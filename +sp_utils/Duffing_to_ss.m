% Duffing_to_ss - Generates a linear state-space model of the Duffing oscillator
% 
% INPUTS:
% - param: Structure containing the alpha, beta, delta and gamma parameters
% - x0: Operating point for the system state
% - u0: Operating point for the control input
% 
% OUTPUTS:
% - sysC: Continuous-time state-space model of the system around the given
%         linearization point (x0, u0).
% 

function sysC = Duffing_to_ss(param, x0, u0)

    A = [-param.delta, -param.alpha - 3*param.beta*x0(2)^2;
         1, 0];
    B = [param.gamma; 0];
    C = [0, 1];
    D = [0];

    sysC = ss(A, B, C, D);

end

