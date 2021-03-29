function [] = include()
% Add this folder and all its subfolders
localDir = fileparts(mfilename('fullpath'));
addpath(fullfile(localDir));

% Add this folder and all its subfolders
path_to_go_to = genpath(localDir);
end