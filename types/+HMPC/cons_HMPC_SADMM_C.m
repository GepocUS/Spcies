%% cons_HMPC_ADMM_C
%
% Generates the constructor for C of the HMPC formulation based on SADMM
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function constructor = cons_HMPC_SADMM_C(recipe)

    constructor = HMPC.cons_HMPC_ADMM_C(recipe);
    
end
