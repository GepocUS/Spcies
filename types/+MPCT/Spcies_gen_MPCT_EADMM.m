%% Function that creates the MPCT controller solved using the EADMM algorithm

function str = Spcies_gen_MPCT_EADMM(target, sys, param, options, save_name)

    %% Compute the ingredients of the controller
    str = MPCT.Spcies_compute_MPCT_EADMM_ingredients(sys, param, options);
    
    %% Call the funciton that constructs the controller
    if strcmp(target, 'Arduino')
        Arduino.gen_MPCT_EADMM_Arduino(str, options, save_name);
    elseif strcmp(target, 'Unity')
        Unity.gen_MPCT_EADMM_Unity(str, options, save_name);
    else
        error('Target not recognized or supported');
    end
    
end
