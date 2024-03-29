%% compute_HMPC_SADMM_split_ingredients
%
% Computes the ingredients of the SADMM-based solver for HMPC which splits
% the decision variables into (z, s) and (z_hat, s_hat).
%
% Information about this formulation can be found at:
%
% P. Krupa, D. Limon, and T. Alamo, “Harmonic based model predictive
% control for set-point tracking", IEEE Transactions on Automatic Control.
% 
% INPUTS:
%   - controller: Contains the information of the controller.
%   - opt: Structure containing options of the solver.
% 
% OUTPUTS:
%   - vars: Structure containing the ingredients required by the solver
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
%

function var = compute_HMPC_SADMM_split_ingredients(controller, opt)

    var = HMPC.compute_HMPC_ADMM_split_ingredients(controller, opt);
    
end
