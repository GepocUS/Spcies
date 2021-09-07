% spcies -- Suite of Predictive Controllers for Industrial Embedded Systems
%
% This toolbox generates sparse solvers for various MPC controllers in differen
% programming languages, including MEX solvers for Matlab.
%
% For information on the toolbox please visit https://github.com/GepocUS/Spcies
%
% This function can be used for various purposes:
%
%   spcies('version')       % Returns the version number of the toolbox. If git is
%                             installed it also returns the git hash in the second
%                             output, i.e., [version, hash] = spcies('version').
%                             The first output is the latest version tag.
%
%   spcies('install')       % Installs the toolbox
%   spcies('uninstall')     % Uninstalls the toolbox
%
%   spcies('test')          % Test the solvers in the toolbox
%

function varargout = spcies(varargin)

if nargin == 0
    help spcies
    return
end

switch varargin{1}
    
    case 'version'
        varargout{1} = 'v0.3.2';
        % If git is installed it will
        try
            [system_status, git_hash] = system('git rev-parse HEAD');
        catch
            
        end
        if system_status == 0
            varargout{2} = convertCharsToStrings(git_hash(1:end-1));
        else
            varargout{2} = 'Could not obtain git hash';
        end
        
    case 'install'
        root_path = spcies_get_root_directory();
        addpath(root_path);
        addpath([root_path '/types/']);
        addpath([root_path '/platforms/']);
        addpath([root_path '/platforms/Matlab/']);
        addpath([root_path '/generated_solvers/']);
        addpath([root_path '/tests/']);
        savepath
        
    case 'uninstall'
        root_path = spcies_get_root_directory();
        warning('off','MATLAB:rmpath:DirNotFound')
        rmpath(root_path);
        rmpath([root_path '/types/']);
        rmpath([root_path '/platforms/']);
        rmpath([root_path '/platforms/Matlab/']);
        rmpath([root_path '/generated_solvers/']);
        rmpath([root_path '/tests/']);
        warning('on','MATLAB:rmpath:DirNotFound')
        savepath
        
    case 'test'
        [varargout{1}, varargout{2}] = spcies_tester(varargin{2:end});
        
    otherwise
        varargout{1} = 'Command not recognized';    
        
end

end
