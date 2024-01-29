%% add_default_options_to_struct
%
% original = add_default_options_to_struct(original, default)
%
% Adds the missing fields from the structure provided in the
% argument 'default' to the structure provided in the 
% argument 'original'.
% 
% It returns the 'original' structure with the new fields added.
% 
% This function is used to fill in the defaults of a structure
% whose fields have not all been provided by the user.
% It does not require knowledge of the fields of the structure.
%
% This function is part of Spcies: https://github.com/GepocUS/Spcies
%

function original = add_default_options_to_struct(original, default)

    fn = fieldnames(default); % Get the field names
    
    for i = 1:numel(fn)
        
        %  Add missing fields
        if ~isfield(original, fn{i})
            original.(fn{i}) = default.(fn{i});
        end
        
        % Fill in empty fields
        if isempty(original.(fn{i}))
            original.(fn{i}) = default.(fn{i});
        end
        
    end
    
end

