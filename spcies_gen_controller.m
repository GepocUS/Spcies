%% spcies_gen_controller - Generation of solvers for embedded MPC
%
% This function generated solvers for the specified MPC controller.
% For a list of the available MPC formulations, please refer to the
% documentation of the toolbox, which can be found at its GitHub
% repository https://github.com/GepocUS/Spcies.
% 
% INPUTS:
%
% All the inputs of this function are passed as name-value pairs.
% This toolbox can be used in conjunction with the GepocToolbox, 
% which is available at https://github.com/GepocUS/GepocToolbox.
% Some of the inputs may be given as classes defined in GepocToolbox.
%
%   - sys: State space model of the system. It should either be an
%          instance of the ssModel class of the GepocToolbox or a
%          structure containing:
%          - .A: matrix A of the state space model.
%          - .B: matrix B of the state space model.
%          It can optionally also contain the following fields:
%          - .x0: operating point for the system state.
%          - .u0: operating point for the system input.
%          - .LBx: Lower bound for the system state.
%          - .UBx: Upper bound for the system state.
%          - .LBu: Upper bound for the system input.
%          - .UBu: Upper bound for the system input.
%          - .Nx: Vector defining the scaling of the system state.
%          - .Nu: Vector defining the scaling of the system input.
%   - param: Structure containing the ingredients of the MPC controller.
%            Its fields will depend on the chosen MPC formulation.
%            Please, refer to the documentation for additional help.
%   - controller: The sys and param arguments can be omitted and instead
%            replaced by this argument, which must be an instance of
%            a subclass of the ssMPC class of the GepocToolbox that is 
%            supported by Spcies.
%   - formulaiton: String that determines the MPC formulation.
%           Currently, it can take the following values:
%           - 'laxMPC': Standard MPC without terminal constraint.
%           - 'equMPC': Standard MPC with terminal equality constraint.
%           - 'MPCT': Model predictive control for tracking (using EADMM).
%           - 'MPCT_ess': MPCT using an extended state-space along ADMM.
%           - 'ellipMPC': Standard MPC with a terminal quadratic constraint.
%           If an empty string is provided, the function tries to determine
%           the formulation automatically based on the other arguments.
%   - solver_options: Structure containing the options of the solver.
%                     Its fields will depend on the chosen MPC formulation.
%                     Please, refer to the documentation for additional help.
%                     Default values are provided, so this argument is optional.
%   - options: Options of the Spcies toolbox. These control aspects
%              such as directories where files are saved, or the
%              name of the saved files. See default_options
%              for a list of the available options. Not all fields
%              need to be provided, since default options are given.
%              The most important of its fields is .platform, which
%              is a string containing the target platform
%              of the code generation routine. Currently, the
%              supported targets are:
%               - 'C': For plain C. The .c and a .h files are generated.
%               - 'Matlab' (default): A mex file is generated.
%   - The fields of the spcies_options argument can also be provided
%     individually as name-value arguments. In such a case, the name-value
%     parameter will take prevalence against the same field of the
%     spcies_options argument if both are provided.
%           
% OUTPUTS:
%   - This function will save files into the directory provided in the 
%     ptions argument.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function spcies_gen_controller(varargin)

    %% Process varargin

    % Extract 'sys' and 'param' from varargin
    [is_sys, indx_sys] = sp_utils.find_in_cell('sys', varargin);
    if ~is_sys
        error("spcies_gen_controller: a 'sys' structure must be provided");
    else
        sys = varargin(indx_sys+1);
        sys = sys{:};
        varargin([indx_sys, indx_sys+1]) = []; % Remove 'sys' from varargin
    end

    [is_param, indx_param] = sp_utils.find_in_cell('param', varargin);
    if ~is_sys
        error("spcies_gen_controller: a 'param' structure must be provided");
    else
        param = varargin(indx_param+1);
        param = param{:};
        varargin([indx_param, indx_param+1]) = []; % Remove 'sys' from varargin
    end

    % Generate options
    options = Spcies_options(varargin{:});
    
    %% Import m files from ./formulations/*
    to_import = strcat(Spcies_options.valid_formulation, '.*');
    import(to_import{:})
    import personal.*
    
    %% Instantiate the Spcies_problem object
    recipe = Spcies_problem(sys, param, options);
    
    %% A formulation must be given
    if isempty(recipe.options.formulation)
        error('Spcies:input_error:no_formulation', 'The formulation field of options is empty. I do not know what to create.');
    end
    
    %% Determine name of constructor function
    
    % Add the formulation
    cons_name = ['cons_' recipe.options.formulation];
    
    % Add the method
    if ~isempty(recipe.options.method)
        cons_name = strcat(cons_name, ['_' recipe.options.method]);
    end
    
    % Add the submethod
    if ~isempty(recipe.options.submethod)
        cons_name = strcat(cons_name, ['_' recipe.options.submethod]);
    end
    
    % Add the platform
    cons_name = strcat(cons_name, ['_' recipe.options.platform]);
    
    %% Instantiate the Spcies_constructor object
    constructor = eval([cons_name '(recipe);']);
    
    %% Construct files
    constructor.construct(recipe.options);

end

