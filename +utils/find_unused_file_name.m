%% find_unused_file_name
%
% Returns a file name that is not currently in use in the specified directory
%
% INPUTS:
%   - dir: Structure containing the fields:
%           - .name: Name of the file (string).
%           - .path: Path where to check if the file exists (string).
%           - .extension: Extension of the file (string).
%
% OUTPUTS:
%    name: A string of the first available name
%
% This function takes 'path/name.extension' and checks if it is a file that currently exists.
% If it does not, then that name is returned. Otherwise, the function returns the
% first available name of the form name_vX, where X is an integer.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function name = find_unused_file_name(dir)

    name = dir.name; % Get name of the file
    
    if isfile([dir.path name '.' dir.extension])
        number = 1;
        new_save_name = [dir.path name '_v' num2str(number)];

        % Check if the file exists and increase the counter if not
        while isfile([new_save_name '.' dir.extension])
            number = number + 1;
            new_save_name = [dir.path name '_v' num2str(number)];
        end

        name = new_save_name; % Update name

    end
    
end

