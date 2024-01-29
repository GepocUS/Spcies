%% Spcies_options - Spcies class for defining and working with the toolbox options
%
% This class contains all the options of the toolbox and methods for working with them,
% getting default options, etc.
% 
% This class is part of Spcies: https://github.com/GepocUS/Spcies
% 

classdef Spcies_options < matlab.mixin.Copyable

    %% PROPERTIES
    properties

        % User selection
        formulation {ischar} % Stores the name of the formulation 
        method {ischar} % Stores the name of the optimization method
        submethod {ischar} % Stores the name of the optimization submethod
        platform {ischar} % Stores the name of the target platform

        % Toolbox options
        verbose % Determines the verbose level of the toolbox (currently unused)

        % General options for code generation
        save_name {ischar} % Stores the name used as seed for file generation
        directory {ischar} % Stores the directory used as seed for file generation
        override % Determines if files with the same name are overwritten
        const_are_static % Determines if const in C are also declared as static variables
        precision {ischar} % Determines if 'double' of 'float' precision is used for real numbers
        inf_value % Determines the number assigned to "infinity"
        save % Determines if files are saved (currently unused, but included for backwards-compatibility for now)

        % General solver options (used in all, or nearly all, solvers)
        debug % Determines if the solver is built in debug-mode (returns additional info to user)
        timing % Determines if the solver measures and returns computation times
        in_engineering % If true, then the solver considers the inputs/outputs to be in engineering units
        time_varying % If true, then the solver considers the system and MPC ingredients to be time-varying
        force_diagonal % Tells the solver to consider ingredients as diagonal if it can

    end

    properties (Hidden=false, SetAccess=protected)

        solver = struct() % Structure that contains solver-specific options, including:
            % k_max % Maximum number of iterations of the solver
            % tol % General exit tolerance of the solver
            % tol_p % Primal exit tolerance of the solver
            % tol_d % Dual exit tolerance of the solver
            % rho % Step-size of the solver (used in ADMM-based solvers)
            % force_vector_rho % If true, then rho is always considered to be a vector (instead of a scalar)
            % alpha % For the SADMM solver
            % sigma % For some HMPC solvers
            % sparse % For some HMPC solvers
            % box_constraints % For some HMPC solvers
            % use_soc % For some HMPC solvers
            % rho_base {mustBeReal} % Step-size used in the EADMM solver
            % rho_mult {mustBeReal} % Step-size used in the EADMM solver

    end

    properties (Constant, Hidden=true)

        % List of valid values for properties which need one
        valid_formulation = {'laxMPC', 'equMPC', 'ellipMPC', 'MPCT', 'HMPC', 'ellipHMPC', 'personal'};
        valid_method = {'ADMM', 'SADMM', 'EADMM', 'FISTA'};
        valid_platform = {'C', 'Matlab'};
        valid_precision = {'double', 'float'};

        % List of methods that are available (supported) for each formulation
        list_accepted_methods = struct(...
            'laxMPC', {{'ADMM', 'FISTA'}},...
            'equMPC', {{'ADMM', 'FISTA'}},...
            'ellipMPC', {{'ADMM'}},...
            'MPCT', {{'ADMM', 'EADMM'}},...
            'HMPC', {{'ADMM', 'SADMM'}},...
            'ellipHMPC', {{'ADMM'}}...
        );

        % List of submethods that are available for each pair formulation-method
        list_accepted_submethods = struct(...
            'laxMPC', struct('ADMM', {{''}}, 'FISTA', {{''}}),...
            'equMPC', struct('ADMM', {{''}}, 'FISTA', {{''}}),...
            'ellipMPC', struct('ADMM', {{'', 'soc'}}),...
            'MPCT', struct('ADMM', {{'cs', 'semiband'}}, 'EADMM', {{''}}),...
            'HMPC', struct('ADMM', {{'cs', 'split'}}, 'SADMM', {{'split'}}),...
            'ellipHMPC', struct('ADMM', {{''}})...
        );

        % List of default method for each submethod
        list_def_methods = struct(...
            'laxMPC', 'ADMM',...
            'equMPC', 'ADMM',...
            'ellipMPC', 'ADMM',...
            'MPCT', 'EADMM',...
            'HMPC', 'ADMM',...
            'ellipHMPC', 'ADMM'...
        );

        % List of default submethods for each pair formulation-method
        list_def_submethods = struct(...
            'laxMPC', struct('ADMM', '', 'FISTA', ''),...
            'equMPC', struct('ADMM', '', 'FISTA', ''),...
            'ellipMPC', struct('ADMM', ''),...
            'MPCT', struct('ADMM', 'cs', 'EADMM', ''),...
            'HMPC', struct('ADMM', '', 'SADMM', 'split'),...
            'ellipHMPC', struct('ADMM', '')...
        );

        % Default values
        def_formulation = '';
        def_method = '';
        def_submethod = '';
        def_platform = 'Matlab';
        def_verbose = 1;
        def_save_name = '';
        def_directory = '$SPCIES$';
        def_override = true;
        def_const_are_static = true;
        def_precision = 'double';
        def_inf_value = 1e6;
        def_save = true;
        def_debug = true;
        def_timing = true;
        def_in_engineering = false;
        def_time_varying = false;
        def_force_diagonal = true;
        def_solver = struct();

    end

    % Deprecated properties
    properties (Dependent, Hidden=true)
        type
        subclass
    end

    methods

        %% CONSTRUCTOR
        function self = Spcies_options(varargin)

            % Parser
            par = inputParser;
            par.CaseSensitive = false;
            par.FunctionName = 'Spcies_options';

            % Name-value parameters
            addParameter(par, 'formulation', Spcies_options.def_formulation);
            addParameter(par, 'method', NaN);
            addParameter(par, 'submethod', NaN);
            addParameter(par, 'platform', Spcies_options.def_platform);
            addParameter(par, 'verbose', Spcies_options.def_verbose);
            addParameter(par, 'options', []); % Structure that can be provided instead
            addParameter(par, 'save_name', NaN);
            addParameter(par, 'directory', NaN);
            addParameter(par, 'override', NaN);
            addParameter(par, 'const_are_static', NaN);
            addParameter(par, 'precision', NaN);
            addParameter(par, 'inf_value', NaN);
            addParameter(par, 'save', NaN);
            addParameter(par, 'debug', NaN);
            addParameter(par, 'timing', NaN);
            addParameter(par, 'in_engineering', NaN);
            addParameter(par, 'time_varying', NaN);
            addParameter(par, 'force_diagonal', NaN);
            % Deprecated:
            addParameter(par, 'type', NaN, @(x) ischar(x)); % Now 'formulation'
            addParameter(par, 'subclass', NaN, @(x) ischar(x)); % Now 'submethod'
            addParameter(par, 'solver_options', []); % Deprecated argument 'solver_options'

            % Parse
            parse(par, varargin{:})

            % Set properties to default values
            self.to_basic();

            % Get formulation
            if ~isempty(par.Results.options) && isfield(par.Results.options, 'formulation')
                    self.formulation = par.Results.options.formulation;
            end
            if isempty(self.formulation)
                self.formulation = par.Results.formulation;
            end

            % Get platform
            if ~isempty(par.Results.options) && isfield(par.Results.options, 'platform')
                    self.platform = par.Results.options.platform;
            end
            if isempty(self.platform)
                self.platform = par.Results.platform;
            end

            % Get verbose
            if ~isempty(par.Results.options) && isfield(par.Results.options, 'verbose')
                    self.verbose = par.Results.options.verbose;
            end
            if isempty(self.verbose)
                self.verbose = par.Results.verbose;
            end

            % Handle deprecated 'type' argument
            if ~isnan(par.Results.type)
                warning("Spcies: 'type' is now deprecated. Please use 'formulation' instead. 'type' will be removed in some future release.")
                self.formulation = par.Results.type;
            end

            % Set the default method if none is given
            if ~isempty(par.Results.options) && isfield(par.Results.options, 'method')
                    self.method = par.Results.options.method;
            end
            if isnan(par.Results.method)
                if isempty(self.method)
                    self.set_default_method();
                end
            else
                self.method = par.Results.method;
            end

            % Set the default submethod if none is given
            if ~isempty(par.Results.options) && isfield(par.Results.options, 'submethod')
                    self.submethod = par.Results.options.submethod;
            end
            if isnan(par.Results.submethod)
                if isempty(self.submethod)
                    self.set_default_submethod();
                end
            else
                self.submethod = par.Results.submethod;
            end
            % Handle deprecated 'subclass' argument
            if ~isnan(par.Results.subclass)
                warning("Spcies: 'subclass' is now deprecated. Please use 'submethod' instead. 'subclass' will be removed in some future release.")
                self.submethod = par.Results.subclass;
            end

            % Get default values for selected formulation and method
            self.set_default();

            % Assign values obtained from 'options'
            if ~isempty(par.Results.options)
                self.set_opt_from_struct(par.Results.options);
            end

            % Assign values obtained from deprecated 'options'
            if ~isempty(par.Results.solver_options)
                warning("Spcies: 'solver_options' is now deprecated. 'options' instead. 'solver_options' will be removed in some future release.")
                self.set_opt_from_struct(par.Results.solver_options);
            end

            % Assign name-value parameters they are not empty
            par_names = fieldnames(par.Results);
            par_names(strcmp(par_names, 'formulation')) = [];
            par_names(strcmp(par_names, 'method')) = [];
            par_names(strcmp(par_names, 'submethod')) = [];
            par_names(strcmp(par_names, 'platform')) = [];
            par_names(strcmp(par_names, 'verbose')) = [];
            par_names(strcmp(par_names, 'type')) = []; % Deprecated
            par_names(strcmp(par_names, 'subclass')) = []; % Deprecated
            par_names(strcmp(par_names, 'options')) = []; % Deprecated
            par_names(strcmp(par_names, 'solver_options')) = []; % Deprecated

            for i = 1:numel(par_names)
                if ~isnan(par.Results.(par_names{i}))
                    self.(par_names{i}) = par.Results.(par_names{i});
                end
            end

            % If no save_name is given, assign the formulation tag
            if isempty(self.save_name)
                self.save_name = self.formulation;
            end

        end

        %% GETTERS AND SETTERS

        function set.formulation(self, value)
            if isempty(value) || sp_utils.find_in_cell(value, Spcies_options.valid_formulation)
                self.formulation = value;
            else
                error("Spcies_options: formulation " + value + " is not supported. Please check Spcies_options.valid_formulation");
            end
        end

        function set.method(self, value)
            if isempty(value) || sp_utils.find_in_cell(value, Spcies_options.valid_method)
                self.method = value;
            else
                error("Spcies_options: method " + value + " is not supported. Please check Spcies_options.valid_method");
            end
        end

        function set.platform(self, value)
            if sp_utils.find_in_cell(value, Spcies_options.valid_platform);
                self.platform = value;
            else
                error("Spcies_options: platform " + value + " is not supported. Please check Spcies_options.valid_platform");
            end
        end

        function set.precision(self, value)
            if sp_utils.find_in_cell(value, Spcies_options.valid_precision);
                self.precision = value;
            else
                error("Spcies_options: precision " + value + " is not supported. Please check Spcies_options.valid_precision");
            end
        end

        function set.save_name(self, value)
            if ischar(value)
                if ~isempty(value)
                    self.save_name = value;
                else
                    self.save_name = self.formulation;
                end
            else
                error("Spcies_options: save_name must be a string of char");
            end
        end

        function set.directory(self, value)
            if ischar(value)
                self.directory = value;
            else
                error("Spcies_options: directory must be a string of char");
            end
        end

        function set.override(self, value)
            if islogical(value) || value == 1 || value == 0
                self.override = value;
            else
                error("Spcies_options: override must be boolean");
            end
        end

        function set.const_are_static(self, value)
            if islogical(value) || value == 1 || value == 0
                self.const_are_static = value;
            else
                error("Spcies_options: const_are_static must be boolean");
            end
        end

        function set.inf_value(self, value)
            if isnumeric(value) && value > 0
                self.inf_value = value;
            else
                error("Spcies_options: inf_value must be >0");
            end
        end

        function set.save(self, value)
            if islogical(value) || value == 1 || value == 0
                self.save = value;
            else
                error("Spcies_options: save must be boolean");
            end
        end

        function set.debug(self, value)
            if islogical(value) || value == 1 || value == 0
                self.debug = value;
            else
                error("Spcies_options: debug must be boolean");
            end
        end

        function set.timing(self, value)
            if islogical(value) || value == 1 || value == 0
                self.timing = value;
            else
                error("Spcies_options: timing must be boolean");
            end
        end

        function set.in_engineering(self, value)
            if islogical(value) || value == 1 || value == 0
                self.in_engineering = value;
            else
                error("Spcies_options: in_engineering must be boolean");
            end
        end

        function set.time_varying(self, value)
            if islogical(value) || value == 1 || value == 0
                self.time_varying = value;
            else
                error("Spcies_options: time_varying must be boolean");
            end
        end

        function set.force_diagonal(self, value)
            if islogical(value) || value == 1 || value == 0
                self.force_diagonal = value;
            else
                error("Spcies_options: force_diagonal must be boolean");
            end
        end

        function set.type(self, value)
            warning("Spcies: 'type' is now deprecated. Please use 'formulation' instead. 'type' will be removed in some future release.")
            self.formulation = value;
        end

        function value = get.type(self)
            value = self.formulation;
        end

        function set.subclass(self, value)
            warning("Spcies: 'subclass' is now deprecated. Please use 'submethod' instead. 'subclass' will be removed in some future release.")
            self.submethod = value;
        end

        function value = get.subclass(self)
            value = self.submethod;
        end

        %% PUBLIC METHODS

        function to_basic(self)
            % to_basic()
            % Resets properties to basic default value
            % Does not modify formulation, method or submethod

            % Remove properties that are not part of the 'basic' options
            prop_names = properties(self);
            prop_names(strcmp(prop_names, 'formulation')) = [];
            prop_names(strcmp(prop_names, 'method')) = [];
            prop_names(strcmp(prop_names, 'submethod')) = [];
            prop_names(strcmp(prop_names, 'platform')) = [];
            prop_names(strcmp(prop_names, 'verbose')) = [];

            % Set options back to basic default values (from Spcies_options.def_prop_name)
            for i = 1:numel(prop_names)
                self.(prop_names{i}) = Spcies_options.(['def_' prop_names{i}]);
            end
        end

        function reset_selection(self)
            % reset_selection()
            % Resets method and submethod to their default values for the current formulation
            self.set_default_method();
            self.set_default_submethod();
        end

        function reset_to_selection(self, formulation, method, submethod)
            % reset_to_selection(formulation, method, submethod)
            % Sets formulation, method and submethod to the given values
            % Then resets the options to default values of the given formulation, method and submethod
            self.to_basic;
            self.set_default_from_selection(formulation, method, submethod);
        end

        function set_default(self)
            % set_default()
            % Sets the default options for the current user-selection
            % This method does not change options that are not in the formulations/ function
            % related to the user-selection
            self.to_default_from_selection(self.formulation, self.method, self.submethod);
        end

        function reset_default(self)
            % reset_default()
            % Resets to the default options for the current user-selection
            % This includes setting all options to their default values
            % This is very similar to reset_all() but it does not affect the user selection
            self.to_basic();
            self.to_default_from_selection(self.formulation, self.method, self.submethod);
        end

        function reset_all(self)
            % reset_all() - Resets everything to the default values for the current formulation
            self.reset_selection();
            self.reset_default();
        end

        function to_default_from_selection(self, formulation, method, submethod)
            % to_default_from_selection(formulation, method, submethod)
            % Assigns the default values of the given formulation, method and submethod.
            % This method searches for a function named 'def_opt_formulation_method_submethod.m'
            % inside subdirectories in formulations/
            % Before that, it resets all the options to their basic values

            % Import all folders in /formulations/
            to_import = strcat(Spcies_options.valid_formulation, '.*');
            import(to_import{:})
            import personal.*

            self.formulation = formulation;
            self.method = method;
            self.submethod = submethod;

            % Construct name of method to be called
            func_name = ['def_options_' formulation];
            if ~isempty(method)
                func_name = strcat(func_name, ['_' method]);
            end
            if ~isempty(submethod)
                func_name = strcat(func_name, ['_' submethod]);
            end

            try
                opt = eval([func_name '(self)']);
                self.set_opt_from_struct(opt);
            catch err
                if strcmp(err.identifier,'MATLAB:UndefinedFunction')
                    if self.verbose > 0
                        warning("no available " + func_name + ". Using general default options.");
                    end
                    self.to_basic();
                else
                    rethrow(err);
                end
            end

        end

        function set_default_method(self)
            % set_default_method()
            % Sets property method to its default value for the current formulation
            if ~isempty(self.formulation) && ~strcmp(self.formulation, 'personal')
                self.method = Spcies_options.get_default_method(self.formulation);
            end
        end

        function set_default_submethod(self)
            % set_default_submethod()
            % Sets property submethod to its default value for the current formulation and method
            if ~isempty(self.formulation) && ~strcmp(self.formulation, 'personal') && ~isempty(self.method)
                self.submethod = Spcies_options.get_default_submethod(self.formulation, self.method);
            end
        end

        function correct = check_method_selection(self)
            % check_method_selection()
            % Returns true if the current method is available for the current formulation
            % Returns false otherwise
            if ~isempty(self.formulation) && ~strcmp(self.formulation, 'personal')
                correct = sp_utils.find_in_cell(self.method, Spcies_options.list_accepted_methods.(self.formulation));
            else
                correct = true;
            end
        end

        function correct = check_submethod_selection(self)
            % check_submethod_selection()
            % Returns true if the current submethod is available for the current formulation and method
            % Returns false otherwise
            if ~isempty(self.formulation) && ~strcmp(self.formulation, 'personal') && ~isempty(self.method)
                correct = sp_utils.find_in_cell(self.submethod, Spcies_options.list_accepted_submethods.(self.formulation).(self.method));
            else
                correct = true;
            end
        end

        function set_opt_from_struct(self, opt, force)
            % set_opt_from_struct(opt, [force=true])
            % Sets the properties of Spcies_options to the ones given in opt.
            % Fields in opt whose name coincides with names of the properties 
            % of Spcies_options will override their value.
            % Fields that do not exist as properties of Spcies_options will
            % instead be saved in Spcies_options.solver
            % If the argument force is set to false, then the field in
            % Spcies_options.solver will only be set if a field with the same
            % name already exists.

            if ~isa(opt, 'struct')
                error("Spcies_option.set_opt_from_struct() only accepts a structure type");
            end
            if nargin == 2
                force = true;
            end

            opt_names = fieldnames(opt);

            % Remove fields that are not allowed to be changed with this method
            opt_names(strcmp(opt_names, 'formulation')) = [];
            opt_names(strcmp(opt_names, 'method')) = [];
            opt_names(strcmp(opt_names, 'submethod')) = [];

            % Assign
            prop_names = properties(self);
            for i = 1:numel(opt_names)
                if sp_utils.find_in_cell(opt_names{i}, prop_names)
                    % Assign the field of opt to the property of Spcies_options
                    self.(opt_names{i}) = opt.(opt_names{i});
                else
                    % Assign value to 'solver' property if it is not a property of Spcies_options
                    if force
                        self.solver.(opt_names{i}) = opt.(opt_names{i});
                    else
                        if sp_utils.find_in_cell(opt_names{i}, fieldnames(self.solver))
                            self.solver.(opt_names{i}) = opt.(opt_names{i});
                        else
                            if self.verbose > 0
                                warning("Spcies_options.set_opt_from_struct() could not find option named " + opt_names{i} + ". Ignoring it.");
                            end
                        end
                    end
                end
            end

        end

        function set(self, varargin)
            % set() - Method used to set the options of the class
            % This is the only method that can be used to modify the
            % fields of property Spcies_options.solver
            % This method can be called in two different ways:
            % 1) Spcies_options.set(opt);
            % where opt is a structure. The values of the fields of
            % opt will be assigned to properties of Spcies_options with
            % the same name. Otherwise, they will be assigned to fields
            % of Spcies_options.solver with the same name (if they exist).

            if nargin==2 && isa(varargin{1}, 'struct')
                self.set_opt_from_struct(varargin{1}, false);
            elseif nargin==3
                if sp_utils.find_in_cell(varargin{1}, properties(self))
                    self.(varargin{1}) = varargin{2};
                else
                    if sp_utils.find_in_cell(varargin{1}, fieldnames(self.solver))
                        self.solver.(varargin{1}) = varargin{2};
                    else
                        if self.verbose > 0
                            warning("Spcies_options.set() could not find option named " + varargin{1} + ". Ignoring it.");
                        end
                    end
                end
            else
                error("Spcies_options.set() only accepts a structure or a name-value pair");
            end

        end

        function force(self, varargin)
            % force()
            % Like Spcies_options.set(), but forcing value assignment even if
            % the field does not exist

            if nargin==2 && isa(varargin{1}, 'struct')
                self.set_opt_from_struct(varargin{1}, true);
            elseif nargin==3
                if sp_utils.find_in_cell(varargin{1}, properties(self))
                    self.(varargin{1}) = varargin{2};
                else
                    self.solver.(varargin{1}) = varargin{2};
                end
            else
                error("Spcies_options.force() only accepts a structure or a name-value pair");
            end

        end

        function defCell = default_defCell(self)
            % default_defCell()
            % Returns the default defCell used by the cond_formulation_method_submethod.m functions.
            % This cell contains the default defines used by the Spcies toolbox for C code-flow.
            defCell = [];
            
            if self.debug
                defCell = sp_utils.add_line(defCell, 'DEBUG', 1, 1, 'bool', 'define');
            end
            if self.timing
                defCell = sp_utils.add_line(defCell, 'MEASURE_TIME', 1, 1, 'bool', 'define');
            end
            defCell = sp_utils.add_line(defCell, 'in_engineering', self.in_engineering, 1, 'int', 'define');
            defCell= sp_utils.add_line(defCell, 'TIME_VARYING', self.time_varying, 1, 'int', 'define');
            if self.force_diagonal
                defCell = sp_utils.add_line(defCell, 'IS_DIAG', 1, 1, 'bool', 'define');
            end

        end

    end % End of public methods

    methods (Static)

        function def_method = get_default_method(formulation)
            % def_method = get_default_method(formulation)
            % Returns the default method for the given formulation
            if isfield(Spcies_options.list_def_methods, formulation)
                def_method = Spcies_options.list_def_methods.(formulation);
            else
                error('Spcies:selection', "No default method for formulation " + formulation);
            end
        end

        function def_submethod = get_default_submethod(formulation, method)
            % def_submethod = get_default_submethod(formulation, method)
            % Returns the default submethod for the given formulation and method
            if isfield(Spcies_options.list_def_submethods, formulation) && isfield(Spcies_options.list_def_submethods.(formulation), method)
                def_submethod = Spcies_options.list_def_submethods.(formulation).(method);
            else
                error('Spcies:selection', "No default submethod for formulation " + formulation + " and method " + method);
            end
        end

    end % End of static methods

end

