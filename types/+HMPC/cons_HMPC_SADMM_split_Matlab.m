%% cons_HMPC_SADMM_split_Matlab
%
% Generates the constructor for Matlab of the SADMM-based solver for the HMPC formulation
% which splits the decision variables into (z, s) and (z_hat, s_hat).
% 
% Information about this formulation can be found at:
%
% P. Krupa, D. Limon, and T. Alamo, “Harmonic based model predictive
% control for set-point tracking", IEEE Transactions on Automatic Control.
% 
% INPUTS:
%   - recipe: An instance of the Spcies_problem class.
%             The specifics of the fields of this recipe can be found cons_HMPC_SADMM_C.m
% 
% OUTPUTS:
%   - constructor: An instance of the Spcies_constructor class ready for file generation.
%                  
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function constructor = cons_HMPC_SADMM_split_Matlab(recipe)

    constructor = HMPC.cons_HMPC_ADMM_split_Matlab(recipe);

end
