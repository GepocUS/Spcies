%% spcies_help -- Documentation provider for Spcies
% 
% spcies_help() or spcies_help('general') prints the general
% help documentation of the Spcies toolbox, including the list
% of available categories.
%
% spcies_help('topic') prints the help associated to 'topic'.
% Help documentation is printed from '/spcies_root/docs/topic.txt'.
% 
% This function can also be accessed as spcies('help', 'topic')
% 

function spcies_help(topic)

    if nargin == 0 || isempty(topic)
        topic = 'general';
    end

    help_file = [spcies_get_root_directory() '/docs/' topic '.txt'];
    try
        type(help_file);
    catch ME
        if strcmp(ME.identifier, 'MATLAB:type:fileNotFound')
            warning(['Spcies help file for topic "' topic '" not found.']);
        else
            rethrow(ME);
        end
    end

end
