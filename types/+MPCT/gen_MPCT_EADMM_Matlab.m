%% gen_laxMPC_ADMM_Matlab
% Generates a MEX file for the ADMM-based solver for the MPCT formulation
% 
% Information about this formulation and the solver can be found at:
% 
% P. Krupa, I. Alvarado, D. Limon, T. Alamo, "Implementation of model predictive control for 
% tracking in embedded systems using a sparse extended ADMM algorithm", arXiv: 2008.09071v1, 2020.
% 
% INPUTS:
%   - vars: Structure containing information needed to declare the variables.
%   - options: Structure containing several options for the solver.
%   - spcies_options: Structure containing several options for the code generation.
% 
% OUTPUT: Saves the controller in the current directory.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function gen_MPCT_EADMM_Matlab(vars, options, spcies_options)

    %% Evaluate function inputs
    def_save_name = 'MPCT_solver';
    
    % Determine the name of the file if it already exists
    if isempty(spcies_options.save_name)
        save_name = def_save_name;
    else
        save_name = spcies_options.save_name;
    end
    spcies_options.save_name = [save_name '_C'];
    
    if ~spcies_options.override
        save_name = utils.find_unused_file_name(save_name, 'c');
    end
    
    %% Create the plain C files
    MPCT.gen_MPCT_EADMM_C(vars, options, spcies_options);
    
    %% Create mex C file
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    
    mex_text = fileread([this_path '/struct_MPCT_EADMM_C_Matlab.c']);
    
    mex_text = strrep(mex_text, "$INSERT_NAME$", save_name); % Insert name of file
    
    %% Create plain C mex file
    mex_file = fopen([spcies_options.directory save_name '.c'], 'wt');
    fprintf(mex_file, mex_text);
    fclose(mex_file);
    
    %% Create the mex file
    eval(['mex ' spcies_options.directory save_name '.c ' spcies_options.directory save_name '_C.c -DCONF_MATLAB -lm']);
    
end
