%% get_full_path
%
% Returns the full path of a dir structure containing:
%   - .name: Name of the file (string).
%   - .path: Path where to check if the file exists (string).
%   - .extension: Extension of the file (string).
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function full_path = get_full_path(the_dir)

    full_path = [the_dir.path the_dir.name '.' the_dir.extension];
    
end

