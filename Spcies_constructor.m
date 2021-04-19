%% 

classdef Spcies_constructor
    
    properties
        files
        data
        flags
        exec_me
        args_exec
    end
    
    methods
        
        % Function that creates a new file field
        function self = new_empty_file(self, name, options, extension)
            
            self.files.(name).dir.name = options.save_name;
            self.files.(name).dir.path = options.directory;
            self.files.(name).dir.extension = extension;
            self.files.(name).save = options.save;
            self.files.(name).override = options.override;
            self.files.(name).blocks = [];
            self.files.(name).data = [];
            self.files.(name).flags = [];
            self.files.(name).appends = [];
            self.files.(name).exec_me = '';
            self.files.(name).args_exec = [];
            
        end
        
        % Function that constructs a program
        function self = construct(self, options)
            
            % Create each one of the files
            fn = fieldnames(self.files);
            
            for i = 1:numel(fn)
                name = fn{i}; % Save name of the field, for convenience
                
                % Process the path of the file
                
                    % Default spcies directory
                if strcmp(self.files.(name).dir.path, '$SPCIES$')
                    self.files.(name).dir.path = [spcies_get_root_directory '/generated_solvers/'];
                end
                
                    % Working directory if empty directory was given
                if isempty(self.files.(name).dir.path)
                    self.files.(name).dir.path = './';
                end
                
                    % Add / at the end if there is none
                if ~strcmp(self.files.(name).dir.path(end), '/')
                    strcat(self.files.(name).dir.path, '/');
                end
                
                    % Determine name of the file
                if ~ self.files.(name).override
                    self.files.(name).dir.name = utils.find_unused_file_name(self.files.(name).dir);
                end
                
                % Initial text
                if ~strcmp(self.files.(name).blocks{1, 1}, '$START$')
                    error('Spcies:constructor:on_start', 'The first entry of Spcies_constructor.files.(name).blocks must be the $START$ file');
                end
                file_text.(name) = fileread(self.files.(name).blocks{1, 2});
                num_blocks = size(self.files.(name).blocks, 1);
                
                % Add the other blocks in the specified order
                for j = 2:num_blocks
                    
                    % Get the text of the block
                    text_block = fileread(self.files.(name).blocks{j, 2});
                    
                    % Add to the main text
                    file_text.(name) = strrep(file_text.(name), self.files.(name).blocks{j, 1}, text_block);
                    
                end
                
                % Add appends
                num_appends = size(self.files.(name).appends, 1);
                for j = 1:num_appends
                    
                    % Get the text of the append
                    text_append = fileread(self.files.(name).appends{j, 2});
                    
                    % Add to the main text
                    file_text.(name) = strcat(file_text.(name),  text_append);
                    
                end
                
                % Add data
                num_data = size(self.files.(name).data, 1);
                for j = 1:num_data
                    
                    % Process the data into a string
                    data_string = utils.declare_variables(self.files.(name).data{j, 2}, options);

                    % Add data to each file_text
                    file_text.(name) = strrep(file_text.(name), self.files.(name).data{j, 1}, data_string);

                end
                
                % Add flags
                num_flags = size(self.files.(name).flags, 1);
                for j = 1:num_flags
                    
                    % Add flag to the main text
                    file_text.(name) = strrep(file_text.(name), self.files.(name).flags{j, 1}, self.files.(name).flags{j, 2});
                    
                end
                
                % Add the name and notice
                file_text.(name) = strrep(file_text.(name), '$INSERT_NAME$', self.files.(name).dir.name);
                file_text.(name) = strrep(file_text.(name), '$INSERT_PATH$', self.files.(name).dir.path);
                file_text.(name) = utils.add_notice(file_text.(name), options);
                
            end
            
            % Add the shared data blocks to the files
            num_data = size(self.data, 1);
            for j = 1:num_data

                % Process the data into a string
                data_string = utils.declare_variables(self.data{j, 2}, options);

                % Add data to each file_text
                for i = 1:numel(fn)
                    file_text.(fn{i}) = strrep(file_text.(fn{i}), self.data{j, 1}, data_string);
                end

            end
            
            % Add the shared flags to the files
            num_flags = size(self.flags, 1);
            for j = 1:num_flags

                % Add data to each file_text
                for i = 1:numel(fn)
                    file_text.(fn{i}) = strrep(file_text.(fn{i}), self.flags{j, 1}, self.flags{j, 2});
                end

            end
            
            % Save the files
            for i = 1:numel(fn)
                
                % Open a file
                file_pointers.(fn{i}) = fopen(utils.get_full_path(self.files.(fn{i}).dir), 'wt');
                
                % Write to file
                fprintf(file_pointers.(fn{i}), file_text.(fn{i}));
                
                % Close file
                fclose(file_pointers.(fn{i}));
                
            end
            
            % Perform the execs of each file
            for i = 1:numel(fn)
                
                if ~isempty(self.files.(fn{i}).exec_me)
                    
                    exec_text = self.files.(fn{i}).exec_me;
                    
                    for j = 1:size(self.files.(fn{i}).args_exec, 1)
                        exec_text = strrep(exec_text, self.files.(fn{i}).args_exec{j, 1}, self.files.(fn{i}).args_exec{j, 2});
                    end
                    
                    exec_text = strrep(exec_text, '$INSERT_NAME$', self.files.(name).dir.name);
                    exec_text = strrep(exec_text, '$INSERT_PATH$', self.files.(name).dir.path);
                    
                    eval(exec_text);
                end
                
            end
            
            % Perform general exec
            if ~isempty(self.exec_me)
                
                exec_text = self.exec_me;
                    
                for j = 1:size(self.files.(fn{i}).args_exec, 1)
                    exec_text = strrep(exec_text, self.args_exec{j, 1}, self.args_exec{j, 1});
                end
                
                eval(exec_text);
                
            end
            
        end
               
    end
    
end
