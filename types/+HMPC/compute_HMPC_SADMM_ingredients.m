%% compute_HMPC_SADMM_ingredients
%
% Computes the ingredients of the SADMM-based solver for HMPC
%
% Information about this formulation can be found at:
%
% P. Krupa, D. Limon, and T. Alamo, “Harmonic based model predictive
% control for set-point tracking", IEEE Transactions on Automatic Control.
% 
% INPUTS:
%   - controller: Contains the information of the controller.
%   - options: Structure containing options of the EADMM solver.
%   - spcies_options: Structure containing the options of the toolbox.
% 
% OUTPUTS:
%   - vars: Structure containing the ingredients required by the solver
%   - vars_nonsparse: Structure containing the ingredients for the non-sparse solver..
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
%

function var = compute_HMPC_SADMM_ingredients(controller, options, spcies_options)

    var = HMPC.compute_HMPC_ADMM_ingredients(controller, options, spcies_options);
    
end
