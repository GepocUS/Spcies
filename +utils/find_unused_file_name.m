%% find_unused_file_name - returns a file name that is not currently in the working directory from a base name
%
% INPUTS:
%   - base_name: A string containing the desired name of the file
%   - extension: A string containing the extension of the file
%
% OUTPUTS:
%    name: A string of the first available name
%
% This function takes 'base_name.extension' and checks if it is a name that is currently used in the
% working directory. If it is not used, then that name is returned. If it is used, then 
% the function returns the firt available name of the form base_name_vX, where X is an integer.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

% Author: Pablo Krupa (pkrupa@us.es)
% 
% Changelog: 
%   v0.1 (2020/12/08): Initial commit version
%

function name = find_unused_file_name(base_name, extension)
    
    name = base_name;
    if isfile([base_name '.' extension])
        number = 1;
        new_save_name = [base_name '_v' num2str(number)];
        while isfile([new_save_name '.' extension])
            number = number + 1;
            new_save_name = [base_name '_v' num2str(number)];
        end
        name = new_save_name;
    end
    
end
