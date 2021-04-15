%% gen_laxMPC_ADMM_Matlab
%
% Generates the MEX file of the ADMM-based solver for the MPCT formulation
% 
% Information about this formulation and the solver can be found at:
% 
% "Implementation of model predictive control for tracking in embedded systems
% using a sparse extended ADMM algorithm", by P. Krupa, I. Alvarado, D. Limon
% and T. Alamo, arXiv preprint: 2008:09071v2, 2020.
% 
% INPUTS:
%   - vars: Structure containing information needed to declare the variables.
%   - options: Structure containing options of the EADMM solver.
%   - spcies_options: Structure containing the options of the toolbox.
% 
% OUTPUT: Saves the mex and any additional files into the appropriate directory.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function gen_MPCT_EADMM_Matlab(vars, options, spcies_options)
    import utils.gen_mex;

    %% Evaluate function inputs
    def_save_name = 'MPCT_solver';
    
    % Determine the name of the file if it already exists
    save_name_mex = utils.process_save_name(spcies_options.save_name, def_save_name, spcies_options.override, 'c');
    spcies_options.save_name = [save_name_mex '_C'];
    
    %% Create the plain C files
    MPCT.gen_MPCT_EADMM_C(vars, options, spcies_options);
    
    %% Create mex C file
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    
    mex_text = fileread([this_path '/struct_MPCT_EADMM_C_Matlab.c']);
    
    mex_text = strrep(mex_text, "$INSERT_NAME$", save_name_mex); % Insert name of file
    
    %% Create plain C mex file
    mex_file = fopen([spcies_options.directory save_name_mex '.c'], 'wt');
    fprintf(mex_file, mex_text);
    fclose(mex_file);
    
    %% Create the mex file
    gen_mex(save_name_mex, spcies_options.directory);
    
end

