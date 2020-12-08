%% Spcies_gen_var_declaration - Generation of variable declaration code
%
% This is a standalone function used to generate the code needed to declare variables in
% the languages supported by Spcies.
% It saves the variable declaration code as a txt file so that it is easy to copy
% and paste into the user's project.
%
% INPUTS:
%   - vars: This a cell with 5 columns and as many rows as variables are to be declared.
%           Each column is a piece of the information needed to declare the variable.
%           - Column 1: Name of the variable (String).
%           - Column 2: Variable from the Matlab workspace to be saves in the target language.
%           - Column 3: Boolean that determines is the variable is initialized (true) or not (false).
%           - Column 4: Type of variable in the target language (String).
%           - Column 5: Additional type of the variable (such as 'constant', 'define', etc).
%   - language: Target language in which to write the variable declaration.
%   - save_name: Optional string ("var_dec"). Determines the name of the file in which to save the code.
%   - override: Optional boolean. Determines if the file is overriden if one with the same name already
%               exists in the working directory.
%
% OUTPUTS: The function generates a .txt file in the working directory.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

% Author: Pablo Krupa (pkrupa@us.es)
% 
% Changelog: 
%   v0.1 (2020/12/08): Initial commit version
%

function Spcies_gen_var_declaration(vars, language, varargin)

    %% Default values
    def_save_name = 'var_dec';
    def_override = true;
    
    %% Parser
    par = inputParser;
    par.CaseSensitive = false;
    par.FunctionName = 'Spcies_gen_var_declaration';
    
    % Required
    addRequired(par, 'vars', @(x) iscell(x));
    addRequired(par, 'language', @(x) ischar(x));
    
    % Name-value parameters
    addParameter(par, 'save_name', def_save_name, @(x) ischar(x));
    addParameter(par, 'override', def_override, @(x) islogical(x) || x==1 || x==0);
    
    % Parse
    parse(par,  vars, language, varargin{:})
    
    %% Generate text containing the variables
    if strcmp(language, 'C')
        var_text = C_code.declareVariables(vars);
        extension = 'c';
    elseif strcmp(language, 'Arduino')
        var_text = Arduino.declareVariables(vars);
        extension = 'ino';
    elseif  strcmp(language, 'Unity')
        var_text = C_code.declareVariables(vars);
        extension = 'XDB';
    else
        error('Spcies:gen_var_declaration:language:unrecognized', 'The language was not recognized as one of the supported languages');
    end
    
    %% Generate txt file
    if ~par.Results.override
        par.Results.save_name = utils.find_unused_file_name(par.Results.save_name, extension);
    end
    my_file = fopen([par.Results.save_name '.' extension], 'wt');
    fprintf(my_file, var_text);
    fclose(my_file);
    
end
