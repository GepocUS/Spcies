%% Spcies_constructor - Spcies class that defines the construction of files
% 
% This class contains the information required to generate the files and
% performs the file generation.
%
% Its constructor is empty, i.e., it generates an instance with empty properties.
%
% PROPERTIES:
%   - files: Structure that contains a field for each file that is generated.
%       This structure contains the following fields:
%         - .dir: Structure. Contains the complete path of the file.
%                 Its fields are: .name, .path and .extension. All strings.
%         - .save: Boolean. Determine if the files should be saved (unused).
%         - .override: Boolean. Determines if previous files are overwritten.
%         - .blocks: Cell of an undetermined number of rows and two columns.
%                    Each row is one of the main parts of the file.
%                    Column 1 is the identifies of the file and column 2 the
%                    path to the file containing the text to be introduced
%                    wherever the identifier is found.
%                    Row 1 is the initial file. Its identifier must be $START$.
%                    The contents of successive files are appended in succession
%                    in the places marked by their respective identifiers.
%         - .data: Cell of an undetermined number of rows and two columns.
%                  Each row is a type of data to be appended in its identifier.
%                  The first column is the identifier and the second is the data.
%                  The data is saved into column 2 as a cell array in the format
%                  required by the declare_variables function of the platform.
%         - .flags: Cell of an undetermined number of rows and two columns.
%                   Each row is a string that must be added in place of its
%                   identifier. Columns 1 is the identifier and 2 is the text.
%         - .append: Cell of an undetermined number of rows and one column.
%                    Each column contains the path to a file whose contents
%                    must be added at the end of the rest of the text.
%                    Similar to the blocks, but without the identifiers.
%         - .exec_me: String that is executed after the file has been saved.
%         - .args_exec: Cell of an undetermined number of rows and two columns.
%                       Arguments added to the exec_me string before it is
%                       executed. The first column is the identifier where the
%                       string of the second column should be added.
%   - data: Same as the files.data field but applied to all the files.
%   - flags: Same as the files.flags field but applied to all the files.
%   - exec_me: Same as the files.exec_me field but executed after the individual
%              exec_me of each file.
%   - args_exec: Same as the files.args_exec, but applied to the property exec_me.
% 
% The identifiers are strings that follow the format '$IDENTIFIER_NAME$'.
% 
% PROPERTIES:
%   - new_empty_file(self, name, options, extension): creates a new field in
%       the field property named 'name'. It also receives the structure
%       Spcies_problem.options of the current problem in 'options'. Finally
%       it receives the extension of the file, to be saved in file.dir.extension.
%
%   - construct(self, options): Creates the files. It receives the structure
%       Spcies_problem.options of the current problem in 'options'.
%       The result of calling this method is the generation of the files.
%
% This class is part of Spcies: https://github.com/GepocUS/Spcies
% 

classdef Spcies_constructor
    
    properties
        files % Files to be generated
        data % Data to be added to the files
        flags % Flags to be substituted by text
        exec_me % Command to be executed after the file is saved
        args_exec % Flags to be added to the exec_me command
    end
    
    methods
      
        %% PROPERTIES 

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % new_empty_file: Function that creates a new file field
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
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % construct: Function that constructs the files
        function self = construct(self, options)

            fn = fieldnames(self.files); % Get the names of the fields of 'files´
            
            % Create the text of each one of the files
            for i = 1:numel(fn)
                name = fn{i}; % Save name of the field, for convenience
                
                % Process the path of the file
                
                    % Default spcies directory
                if strcmp(self.files.(name).dir.path, '$SPCIES$')
                    self.files.(name).dir.path = [spcies_get_root_directory '/generated_solvers/'];
                end
                
                    % Default spcies directory if empty directory was given
                if isempty(self.files.(name).dir.path)
                    self.files.(name).dir.path = [spcies_get_root_directory '/generated_solvers/'];
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
                file_text.(name) = fileread(self.files.(name).blocks{1, 2}); % Read the file containing the initial text

                % Add the other blocks in the order in which they appear
                num_blocks = size(self.files.(name).blocks, 1); % Get the number of blocks
                for j = 2:num_blocks
                    
                    % Get the text of the block
                    text_block = fileread(self.files.(name).blocks{j, 2});
                    
                    % Add to the main text
                    file_text.(name) = strrep(file_text.(name), self.files.(name).blocks{j, 1}, text_block);
                    
                end
                
                % Add appends at the end of the file
                num_appends = size(self.files.(name).appends, 1); % Get the number of appends
                for j = 1:num_appends
                    
                    % Get the text of the append
                    text_append = fileread(self.files.(name).appends{j, 1});
                    
                    % Add to the main text
                    file_text.(name) = strcat(file_text.(name),  text_append);
                    
                end
                
                % Add data to the places marked by their identifiers
                num_data = size(self.files.(name).data, 1); % Get the number of data types
                for j = 1:num_data
                    
                    % Process the data into a string
                    data_string = utils.declare_variables(self.files.(name).data{j, 2}, options);

                    % Add data to each file_text
                    file_text.(name) = strrep(file_text.(name), self.files.(name).data{j, 1}, data_string);

                end
                
                % Add flags to the places marked by their identifiers
                num_flags = size(self.files.(name).flags, 1); % Get the number of flags
                for j = 1:num_flags
                    
                    % Add flag to the main text
                    file_text.(name) = strrep(file_text.(name), self.files.(name).flags{j, 1}, self.files.(name).flags{j, 2});
                    
                end
                
                % Add the default flags and appends
                file_text.(name) = strrep(file_text.(name), '$INSERT_NAME$', self.files.(name).dir.name); % Name of the file
                file_text.(name) = strrep(file_text.(name), '$INSERT_PATH$', self.files.(name).dir.path); % Path of the file
                file_text.(name) = utils.add_notice(file_text.(name), options); % Notice saying that is was generated by Spcies
                
            end
            
            % Add the shared data blocks to the files
            num_data = size(self.data, 1); % Get the number of shared data types
            for j = 1:num_data
                
                % Process the data into a string
                data_string = utils.declare_variables(self.data{j, 2}, options);
                num_data = length(data_string);

                % Add data to each file_text
                for i = 1:numel(fn)
                    for k = 1:num_data-1
                        file_text.(fn{i}) = strrep(file_text.(fn{i}), self.data{j, 1}, strcat(data_string{k}, self.data{j, 1}));
                    end
                    file_text.(fn{i}) = strrep(file_text.(fn{i}), self.data{j, 1}, data_string{num_data});
                end

            end
            
            % Add the shared flags to the files
            num_flags = size(self.flags, 1); % Get the number of flags shared by all files
            for j = 1:num_flags

                % Add data to each file_text
                for i = 1:numel(fn)
                    file_text.(fn{i}) = strrep(file_text.(fn{i}), self.flags{j, 1}, self.flags{j, 2});
                end

            end
            
            % Save the files
            for i = 1:numel(fn)
                
                % Open file
                file_pointers.(fn{i}) = fopen(utils.get_full_path(self.files.(fn{i}).dir), 'wt');
                
                % Write to file
                fprintf(file_pointers.(fn{i}), file_text.(fn{i}));
                
                % Close file
                fclose(file_pointers.(fn{i}));
                
            end
            
            % Perform the execs of each file
            for i = 1:numel(fn)
                
                if ~isempty(self.files.(fn{i}).exec_me)
                    
                    % Get the initial text to be executed
                    exec_text = self.files.(fn{i}).exec_me;
                    
                    % Add the args in place of the identifiers
                    for j = 1:size(self.files.(fn{i}).args_exec, 1)
                        exec_text = strrep(exec_text, self.files.(fn{i}).args_exec{j, 1}, self.files.(fn{i}).args_exec{j, 2});
                    end
                    
                    % Add the default flags
                    exec_text = strrep(exec_text, '$INSERT_NAME$', self.files.(name).dir.name); % Name of file
                    exec_text = strrep(exec_text, '$INSERT_PATH$', self.files.(name).dir.path); % Path to file
                    
                    % Execute command
                    warning('off','MATLAB:mex:GccVersion_link') % Dissable warning of unsupported gcc compiler.
                    eval(exec_text);
                    warning('on','MATLAB:mex:GccVersion_link') % Enable the warning

                end
                
            end
            
            % Execute general command
            if ~isempty(self.exec_me)
                
                % Get the initial text to be executed
                exec_text = self.exec_me;
                    
                % Add the args in place of the identifiers
                for j = 1:size(self.args_exec, 1)
                    exec_text = strrep(exec_text, self.args_exec{j, 1}, self.args_exec{j, 1});
                end
                
                % Execute command
                eval(exec_text);
                
            end
            
        end
               
    end
    
end

