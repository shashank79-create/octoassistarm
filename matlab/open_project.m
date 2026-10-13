function open_project()
folder=fileparts(fileparts(mfilename('fullpath')));[sim,client]=octo_connect(); %#ok<ASGLU>
octo_prepare_stopped(sim);
sim.loadScene(fullfile(folder,'scene','OctoAssist.ttt'));
end
