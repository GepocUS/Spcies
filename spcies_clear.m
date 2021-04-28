%% spcies_clear - clears the folder $SPCIES$/generated_solvers
%
% It deletes all the temp files in $SPCIES$/generated_solvers
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function spcies_clear()

    %% Extensions to be eliminated
    extensions = {'c', 'h'};
    
    %% Path to the folder
    the_folder = [spcies_get_root_directory '/generated_solvers/'];
    
    for i = 1:length(extensions)
        file_pattern = fullfile(the_folder, ['*.' extensions{i}]);
        the_files = dir(file_pattern);
        for j = 1:length(the_files)
            base_file_name = the_files(j).name;
            full_file_name = fullfile(the_folder, base_file_name);
            delete(full_file_name);
        end
    end
    
end
