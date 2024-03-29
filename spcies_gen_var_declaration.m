%% spcies_gen_var_declaration - Generation of variable declaration code
%
% This is a standalone function used to generate the code needed to 
% declare variables in the programming languages supported by Spcies.
% It saves the variable declaration code as a txt file so that it can
% directly copied into the source code.
%
% INPUTS:
%   - vars: This a cell with 5 columns and as many rows as variables are
%           to be declared. Each column is a piece of the information
%           required to declare the variable.
%           - Column 1: Name of the variable (String).
%           - Column 2: Variable from the Matlab workspace to be saved
%                       in the target language.
%           - Column 3: Boolean that determines is the variable is
%                       initialized (true) or not (false).
%           - Column 4: Type of variable in the target language (String).
%           - Column 5: Additional information needed to declare the 
%                       variable. it will depend on the language.
%   - language: String that determines the target language in which to
%               write the variable declaration. Currently, the supported
%               languages are:
%               - 'C': For plain C.
%               - 'Arduino': Specifically for Arduino. very similar to 'C'.
%               - 'Unity': For Unity Pro XL (IDE for Schneider PLCs).
%   - save_name: Optional string that determines the name of the file in
%                which the code declaration is written.
%                It defaults to 'var_dec'.
%   - override: Optional boolean. Determines if the file is overwritten if
%               it already exists in the working directory.
%
% OUTPUTS: The function generates a .txt file in the working directory.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function spcies_gen_var_declaration(vars, language, varargin)

    %% Default values
    def_save_name = 'var_dec';  % Default value of the save_name argument
    def_directory = './'; % Default value of the directory where to save files
    def_override = true; % Default value of the option that determines if files are overwritten
    
    %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'spcies_gen_var_declaration';
    
    % Required
    addRequired(par, 'vars', @(x) iscell(x));
    addRequired(par, 'language', @(x) ischar(x));
    
    % Name-value parameters
    addParameter(par, 'save_name', def_save_name, @(x) ischar(x));
    addParameter(par, 'directory', def_directory, @(x) ischar(x));
    addParameter(par, 'override', def_override, @(x) islogical(x) || x==1 || x==0);
    
    % Parse
    parse(par,  vars, language, varargin{:})
    directory = par.Results.directory;
    
    % Check arguments
    if isempty(directory)
        directory = def_directory;
    end
    
    %% Generate text containing the variables
    if strcmp(language, 'C')
        var_text = C_code.declare_variables(vars);
        extension = 'c';
    elseif strcmp(language, 'Arduino')
        var_text = Arduino.declare_variables(vars);
        extension = 'ino';
    elseif  strcmp(language, 'Unity')
        var_text = Unity.declare_variables(vars);
        extension = 'XDB';
    else
        error('Spcies:gen_var_declaration:language:unrecognized', 'The language was not recognized as one of the supported languages');
    end
    
    %% Generate txt file
    the_dir.name = par.Results.save_name;
    the_dir.path = directory;
    the_dir.extension = 'txt';
    
    if ~par.Results.override
        the_dir.save_name = sp_utils.find_unused_file_name(the_dir);
    end
    my_file = fopen(sp_utils.get_full_path(the_dir), 'wt');
    for i = 1:length(var_text)
        fprintf(my_file, var_text{i});
    end
    fclose(my_file);
    
end

