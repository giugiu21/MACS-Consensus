function add_matlab_paths()
% add all reusable Continuous Linear System folders to the MATLAB path

root_dir = fileparts(mfilename('fullpath'));

addpath(root_dir);
addpath(fullfile(root_dir, 'agents'));
addpath(fullfile(root_dir, 'graph'));
addpath(fullfile(root_dir, 'control'));
addpath(fullfile(root_dir, 'dynamics'));
addpath(fullfile(root_dir, 'triggers'));
addpath(fullfile(root_dir, 'simulation'));
addpath(fullfile(root_dir, 'analysis'));
%addpath(fullfile(root_dir, 'visualization'));
end
