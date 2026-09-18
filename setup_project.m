function setup_project()
%SETUP_PROJECT Add the project code and asset folders to MATLAB's path.
root=fileparts(mfilename('fullpath'));
folders={'setup','controllers','training','experiments','analysis', ...
    'plotting','model_construction','models','assets'};
addpath(fullfile(root,'Codes'));
for k=1:numel(folders)
    addpath(fullfile(root,'Codes',folders{k}));
end
% Retain the existing working-directory convention for generated Results.
cd(fullfile(root,'Codes'));
end
