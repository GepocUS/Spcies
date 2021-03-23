%% Spcies_default_options - Returns the default options of the Spcies toolbox
% 
% This function containts the default options of the Spcies toolbox.
% It returns them in a structure that can be used as the argument
% spcies_options of other functions of the toolbox.
% 
% The options of the toolbox are:
%   - target: String that determines the target embedded system.
%   - save_name: String that indicates the name with which files are saved.
%                If empty, then each controller will use its own
%                default naming convention.
%   - directory: String that indicates the directory where files are saved.
%   - override: Boolean that determines if files can be overwritten.  
%               If they cannot, then they are saved with a number appended
%               to the end of the file.
%   - force_vector_rho: Some solvers use a penalty parameter named rho.
%               This penalty is usually a scalar, but this boolean
%               option forces the use of an array if set to true.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function def_spcies_opt = Spcies_default_options()
    
    % Default options
    def_target = 'Matlab'; % Default target
    def_save_name = ''; % Default value of the save_name argument
    def_directory = './'; % Default directory where to save files
    def_override = true; % Determined if files are overwritten if one with the same name already exists
    def_force_vector_rho = false; % If true, forces the penalty parameter rho to be defined as a vector

    % Structure containing the default options
    def_spcies_opt = struct('target', def_target, 'save_name', def_save_name,...
                            'directory', def_directory, 'override', def_override,....
                            'force_vector_rho', def_force_vector_rho);

end

