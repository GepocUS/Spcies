%% Function for generating controllers

function str = Spcies_gen_controller(type, target, sys, param, options, save_name)

if strcmp(type, 'MPCT_EADMM')
    str = MPCT.Spcies_gen_MPCT_EADMM(target, sys, param, options, save_name);
else
    error('Type not recognized or supported');
end

end
