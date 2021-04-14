

function struct_text = get_generic_solver_struct()

    full_path = mfilename('fullpath');
    this_path = fileparts(full_path);
    
    struct_text = fileread([this_path '/generic_solver_struct.c']);

end
