%% cons_MPCT_ADMM_cs_Matlab
%
% Generates the constructor for Matlab of the MPCT solver using ADMM on an extended state space
% 
% The solver extends the state and control inputs by adding the artificial reference to them.
% Currently, there is no additional documentation available for the solver.
% 
% INPUTS:
%   - recipe: An instance of the Spcies_problem class.
%             The specifics of the fields of this recipe can be found in cons_MPCT_ADMM_cs_C.m
% 
% OUTPUTS:
%   - constructor: An instance of the Spcies_constructor class ready for file generation.
%                  
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function constructor = cons_MPCT_ADMM_cs_Matlab(recipe)

    %% Add a name
    if isempty(recipe.options.save_name)
        recipe.options.save_name = recipe.options.formulation;
    end

    %% Construct the C files
    recipe_C = recipe;
    recipe_C.options.platform = 'C';
    recipe_C.options.directory = '$SPCIES$';
    recipe_C.options.override = true;
    recipe_C.options.save_name = [recipe.options.save_name '_plain_C'];
    recipe_C.options.save = false;
    
    constructor_C = MPCT.cons_MPCT_ADMM_cs_C(recipe_C);
    constructor_C = constructor_C.construct(recipe_C.options);
    
    %% Generate constructor
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    
    % Declare empty constructor
    constructor = Spcies_constructor;
    
    % Add mex file code
    constructor = constructor.new_empty_file('mex_code', recipe.options, 'c');
    constructor.files.mex_code.blocks = {'$START$', [this_path '/struct_MPCT_ADMM_cs_C_Matlab.c']};
    constructor.files.mex_code.flags = {'$C_CODE_NAME$', constructor_C.files.code.dir.name};
    
    % Add the execution command for compiling the mex file
    constructor.files.mex_code.exec_me = sp_utils.get_generic_mex_exec(recipe.options);
    constructor.files.mex_code.args_exec = {'$C_CODE_PATH$', constructor_C.files.code.dir.path;...
                                            '$C_CODE_NAME$', constructor_C.files.code.dir.name};
    
end

