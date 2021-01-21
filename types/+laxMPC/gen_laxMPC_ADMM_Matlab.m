%% gen_laxMPC_ADMM_Matlab
% Generates a MEX file for the ADMM-based solver for for the lax MPC formulation
% 
% Information about this formulation and the solver can be found at:
% 
% P. Krupa, D. Limon, T. Alamo, "Implementation of model predictive control in
% programmable logic controllers", Transactions on Control Systems Technology, 2020.
% 
% INPUTS:
%   - vars: Structure containing information needed to declare the variables.
%   - options: Structure containing several options for the solver.
%   - save_name: String containing the name of the file the controller is saved to.
%   - override: Boolean that determines is the controller is overriden if the file already exists.
% 
% OUTPUT: Saves the controller into a txt file in the current directory.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function gen_laxMPC_ADMM_Matlab(vars, options, spcies_options)

    %% Evaluate function inputs
    def_save_name = 'laxMPC';

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
    laxMPC.gen_laxMPC_ADMM_C(vars, options, spcies_options);
    
    %% Create mex C file
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    
    mex_text = fileread([this_path '/struct_laxMPC_ADMM_C_Matlab.c']);
    
    mex_text = strrep(mex_text, "$INSERT_NAME$", save_name); % Insert name of file
    
    %% Create plain C mex file
    mex_file = fopen([spcies_options.directory save_name '.c'], 'wt');
    fprintf(mex_file, mex_text);
    fclose(mex_file);
    
    %% Create the mex file
    eval(['mex ' spcies_options.directory save_name '.c ' spcies_options.directory save_name '_C.c -DCONF_MATLAB -lm']);
    
end
