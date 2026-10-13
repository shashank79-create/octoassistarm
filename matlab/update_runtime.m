function update_runtime()
% Deploy source changes into the SAME scene; no new scene variants.
folder=fileparts(fileparts(mfilename('fullpath')));[sim,client]=octo_connect(); %#ok<ASGLU>
octo_prepare_stopped(sim);
runtime=octo_num(sim.getObject('/OctoAssist/OctoAssistRuntime'));
sim.setStringProperty(runtime,'code',octo_runtime_code(folder));
sim.saveScene(fullfile(folder,'scene','OctoAssist.ttt'));
end
