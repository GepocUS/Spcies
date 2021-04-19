%%

function spcies_path = spcies_get_root_directory()

    % Get the path to the directory where this function is saved
    full_path = mfilename('fullpath');
    spcies_path = fileparts(full_path);
    
end
