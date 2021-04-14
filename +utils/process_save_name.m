%% process_save_name
%
% This function processes names for files
% 
% INPUTS:
%   - save_name: base name of the file (string).
%   - default_save_name: default name of the file (string).
%                        Used if save_name is empty.
%   - override: Determines if numbers are appended to the name
%                to avoid overwritting existing files (boolean).
%   - extension: Extension of the file (string, defaults to 'c').
%   - append: Text added at the end of the processed name, after
%             additional numbering (string, defaults to '').
%
% OUTPUT:
%   - save_name: Processed save_name (string).
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function save_name = process_save_name(save_name, default_save_name, override, extension, append)

    % Determine extension and append
    if nargin == 3
        extension = 'c';
        append = '';
    elseif nargin == 4
        append = '';
    end
    
    % Use default value if no name is provided
    if isempty(save_name)
        save_name = default_save_name;
    end
    
    % Determine if file is overwritten or not
    if ~override
        save_name = utils.find_unused_file_name(save_name, extension);
    end
    
    % Add the append
    save_name = [save_name append];
    
end
