% SPCIES -- Suite of Predictive Controllers for Industrial Embedded Systems
%
% This toolbox generates sparse solvers for various MPC controllers in different
% programming languages, including MEX solvers for Matlab.
%
% For information on the toolbox please visit https://github.com/GepocUS/Spcies
%
% This function can be used for various purposes (characters in [] indicate an
% alternative way of calling the function, e.g., "spcies('h')"):
%
% > spcies('[h]elp')        % For the general help and the doc pages
%                             Use spcies('help', 'topic') for displaying the help
%                             page of 'topic'.
%                             Use spcies('help') for general information and the
%                             list of general topics.
%                             Alternative call: spcies('h') or spcies('h', 'topic')
% > spcies('[gen]erate')    % For generating an MPC controller. See spcies_gen_controller()
% > spcies('install')       % Installs the toolbox.
% > spcies('uninstall')     % Uninstalls the toolbox.
% > spcies('[c]lear')       % Deletes the contents of 'generated_solvers/'
% > spcies('root')          % Returns the directory where Spcies is installed
% > spcies('[l]icense')     % Prints the license
% > spcies('[v]ersion')     % Returns the version number of the toolbox. If git is
%                             installed it also returns the git hash in the second
%                             output, i.e., [version, hash] = spcies('version').
%                             The first output is the latest version tag.
% > spcies('[t]est')        % Tests the solvers in the toolbox.
% 
% This function is part of Spcies: https://github.com/GepocUS/Spcies
% 

function varargout = spcies(varargin)

if nargin == 0
    help spcies
    return
end

switch varargin{1}
    
    % [version, git_hash] = spcies('version') or spcies('v')
    %
    % Returns information of the version of the toolbox.
    % 
    % OUTPUTS:
    %   - version: Version number of the toolbox -> v.MAJOR.MINOR.PATCH
    %   - git_hash: Current git hash of the toolbox. Only works if git
    %               is installed and the toolbox is being tracked.
    % 
    case {'version', 'v'}
        
        varargout{1} = 'v0.3.11';
        
        % If git is installed it will return the hash of the current commit
        try
            [system_status, git_hash] = system('git rev-parse HEAD');
        catch
            varargout{2} = 'Could not obtain git hash';
        end
        
        if system_status == 0
            varargout{2} = convertCharsToStrings(git_hash(1:end-1));
        else
            varargout{2} = 'Could not obtain git hash';
        end
        
    % spcies('install')
    %
    % Installs the toolbox. It adds the required directories to the
    % Matlab path.
    %
    case 'install'
        root_path = spcies_get_root_directory();
        addpath(root_path);
        addpath([root_path '/formulations/']);
        addpath([root_path '/classes/']);
        addpath([root_path '/platforms/']);
        addpath([root_path '/platforms/Matlab/']);
        addpath([root_path '/platforms/Matlab/personal/']);
        addpath([root_path '/generated_solvers/']);
        addpath([root_path '/tests/']);
        savepath
        disp("Spcies: successfully installed");
    
    % spcies('uninstall')
    %
    % Uninstalls the toolbox. It removes the required directories from
    % the Matlab path.
    %
    case 'uninstall'
        root_path = spcies_get_root_directory();
        warning('off','MATLAB:rmpath:DirNotFound')
        rmpath(root_path);
        rmpath([root_path '/formulations/']);
        rmpath([root_path '/classes/']);
        rmpath([root_path '/platforms/']);
        rmpath([root_path '/platforms/Matlab/']);
        rmpath([root_path '/platforms/Matlab/personal/']);
        rmpath([root_path '/generated_solvers/']);
        rmpath([root_path '/tests/']);
        warning('on','MATLAB:rmpath:DirNotFound')
        savepath
        disp("Spcies: successfully uninstalled");

    % spcies('generate', args) or spcies('gen', args)
    % 
    % Main functionality of the Spcies toolbox.
    % This is an entry point to spcies_gen_controller(), which
    % generates the solver for an MPC formulation.
    % Execute 'help spcies_gen_controller' for information about
    % its imputs and outputs.
    % The args given to "spcies('generate', args)" are passed
    % as spcies_gen_controller(args)
    % 
    case {'generate', 'gen'}
        spcies_gen_controller(varargin{2:end});
    
    % spcies('test') or spcies('t')
    %
    % Tests the solvers in the toolbox.
    %
    % This function calls the function /tests/spcies_tester.m with its default
    % settings. Please refer to its documentation for further details.
    %
    case 'test'
        [varargout{1}, varargout{2}] = spcies_tester(varargin{2:end});

    % spcies('clear')
    % 
    % Deletes the contents of /generated_solvers/
    % Calls spcies_clear();
    %
    case 'clear'
        spcies_clear(varargin{2:end});

    % spcies('root')
    % 
    % Returns the path to the directory where Spcies is installed
    % Calls spcies_get_root_directory();
    %
    case 'root'
        varargout{:} = spcies_get_root_directory(varargin{2:end});

    % spcies ('help') or spcies('h')
    % 
    % Help pages of the toolbox.
    % 
    % spcies('help') - shows the general 'help' info, including the category list
    % spcies('help', 'topic') - shows the 'topic' help
    % 
    case {'help', 'h'}
        spcies_help(varargin{2:end});

    % spcies('license')
    % 
    % Prints the license
    % 
    case 'license'
        try
            type([spcies_get_root_directory() '/LICENSE']);
        catch ME
            if strcmp(ME.identifier, 'MATLAB:type:fileNotFound')
                warning(['Spcies license file not found.']);
            else
                rethrow(ME);
            end
        end
    
    % Command not recognized
    otherwise
        error("spcies: command '" + varargin{1} + "' not recognized");
        % varargout{1} = 'spcies: command not recognized';    
        
end

end
