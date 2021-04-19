%% add_default_options_to_struct

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
