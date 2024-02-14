%% spcies_get_root_directory - Return installation directory
%
% This function returns the directory where the Spcies toolbox
% is located and installed.
% The main purpose of this function is to be used by the toolbox
% itself to locate the directories where its resources are located.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function spcies_path = spcies_get_root_directory()

    % Get the path to the directory where this function is saved
    full_path = mfilename('fullpath');
    spcies_path = fileparts(full_path);
    
end

