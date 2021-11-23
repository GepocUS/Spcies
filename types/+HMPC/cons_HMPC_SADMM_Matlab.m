%% cons_HMPC_SADMM_Matlab
%
% Generates the constructor for Matlab of the SADMM-based solver for the HMPC formulation.
% 
% Information about this formulation can be found at:
%
% P. Krupa, D. Limon, and T. Alamo, “Harmonic based model predictive
% control for set-point tracking", IEEE Transactions on Automatic Control.
%
% Information about the solver itself will be available shortly.
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

function constructor = cons_HMPC_SADMM_Matlab(recipe)

    constructor = HMPC.cons_HMPC_ADMM_Matlab(recipe);

end
