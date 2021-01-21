%%  This function returns the default options of the Spcies toolbox
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function def_spcies_opt = Spcies_default_options()
    
    % Defuault options
    def_target = 'Matlab'; % Default target
    def_save_name = ''; % Default value of the save_name argument
    def_directory = './'; % Default value of the directory where to save files
    def_override = true; % Default value of the option that determines if files are overwritten

    % Structure containing the default options
    def_spcies_opt = struct('target', def_target, 'save_name', def_save_name,...
                            'directory', def_directory, 'override', def_override);

end
