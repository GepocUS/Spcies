%% gen_MPCT_extended_ss_ADMM_Matlab
%
% Generates the MEX file of the MPCT solver using ADMM on an extended state space
% 
% The solver extends the state and control inputs by adding the artificial reference to them.
% Currently, there is no additional documentation available for the solver.
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

function gen_MPCT_extended_ss_ADMM_Matlab(vars, options, spcies_options)
    import utils.gen_mex;

    %% Evaluate function inputs
    def_save_name = 'MPCT_solver';
    
    % Determine the name of the file if it already exists
    save_name_mex = utils.process_save_name(spcies_options.save_name, def_save_name, spcies_options.override, 'c');
    spcies_options.save_name = [save_name_mex '_C'];
    
    %% Create the plain C files
    MPCT.gen_MPCT_extended_ss_ADMM_C(vars, options, spcies_options);
    
    %% Create mex C file
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    
    mex_text = fileread([this_path '/struct_MPCT_extended_ss_ADMM_C_Matlab.c']);
    
    mex_text = strrep(mex_text, "$INSERT_NAME$", save_name_mex); % Insert name of file
    
    %% Create plain C mex file
    mex_file = fopen([spcies_options.directory save_name_mex '.c'], 'wt');
    fprintf(mex_file, mex_text);
    fclose(mex_file);
    
    %% Create the mex file
    gen_mex(save_name_mex, spcies_options.directory);
    
end

