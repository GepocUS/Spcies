%% cons_HMPC_ADMM_Matlab
%
% Generates the constructor for Matlab of the ADMM-based solver for the HMPC formulation.
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
%             The specifics of the fields of this recipe can be found cons_HMPC_ADMM_C.m
% 
% OUTPUTS:
%   - constructor: An instance of the Spcies_constructor class ready for file generation.
%                  
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function constructor = cons_HMPC_ADMM_Matlab(recipe)

    %% Add a name
    if isempty(recipe.options.save_name)
        recipe.options.save_name = recipe.options.type;
    end

    %% Construct the C files
    recipe_C = recipe;
    recipe_C.options.platform = 'C';
    recipe_C.options.directory = '$SPCIES$';
    recipe_C.options.override = true;
    recipe_C.options.save_name = [recipe.options.save_name '_plain_C'];
    recipe_C.options.save = false;
    
    constructor_C = HMPC.cons_HMPC_ADMM_C(recipe_C);
    constructor_C = constructor_C.construct(recipe_C.options);
    
    %% Generate constructor
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    
    % Declare empty constructor
    constructor = Spcies_constructor;
    
    % Add mex file code
    constructor = constructor.new_empty_file('mex_code', recipe.options, 'c');
    constructor.files.mex_code.blocks = {'$START$', [this_path '/struct_HMPC_ADMM_C_Matlab.c']};
    constructor.files.mex_code.flags = {'$C_CODE_NAME$', constructor_C.files.code.dir.name};
    
    % Add the execution command for compiling the mex file
    constructor.files.mex_code.exec_me = utils.get_generic_mex_exec(recipe.options);
    constructor.files.mex_code.args_exec = {'$C_CODE_PATH$', constructor_C.files.code.dir.path;...
                                            '$C_CODE_NAME$', constructor_C.files.code.dir.name};
    
end
