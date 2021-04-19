%% get_generic_solver_struct
% 
% This function returns the text of the generic structure
% of the code for solvers in C.
%
% The text is loaded from the file ./generic_solver_struct.c
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function path_to_file = get_generic_solver_struct()

    % Get the path to the directory where this function is saved
    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    
    path_to_file = [this_path '/generic_solver_struct.c'];
    
    % Read the file ./generic_solver_struct.c
    % struct_text = fileread([this_path '/generic_solver_struct.c']);

end
