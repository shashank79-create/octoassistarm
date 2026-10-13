function build_project()
folder=fileparts(fileparts(mfilename('fullpath')));[sim,client]=octo_connect(); %#ok<ASGLU>
if ~isfolder(fullfile(folder,'scene')),mkdir(fullfile(folder,'scene'));end
if ~isfolder(fullfile(folder,'logs')),mkdir(fullfile(folder,'logs'));end
assert(octo_num(sim.getSimulationState())==octo_num(sim.simulation_stopped),'Stop the simulation first.');
backup=fullfile(fileparts(fileparts(folder)),'work','scene_before_octofresh.ttt');
if ~isfile(backup),sim.saveScene(backup);end
sim.loadScene('C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu/system/dfltscn.ttt');
s=octo_config();s.homeQ=octo_ik(s.homePalm,s);
script=octo_num(sim.createScript(sim.scripttype_customization,fileread(fullfile(folder,'runtime','build_scene.lua')),0,'lua'));
sim.setObjectAlias(script,'TemporaryFreshBuilder');sim.initScript(script);
info=sim.callScriptFunction('buildFresh',script,jsonencode(s));sim.removeObjects({script});
runtime=octo_num(sim.createScript(sim.scripttype_simulation,octo_runtime_code(folder),0,'lua'));
sim.setObjectAlias(runtime,'OctoAssistRuntime');sim.setObjectParent(runtime,info.root,true);
sim.writeCustomStringData(info.root,'OctoSpec',jsonencode(s));sim.writeCustomStringData(info.root,'OctoHandles',jsonencode(info));
sim.setFloatParam(sim.floatparam_simulation_time_step,s.dt);
sim.setBoolProperty(sim.handle_scene,'realtimeSimulation',false);
sim.setIntProperty(sim.handle_scene,'stepsPerRendering',10);
sim.saveScene(fullfile(folder,'scene','OctoAssist.ttt'));
fid=fopen(fullfile(folder,'geometry.json'),'w');fwrite(fid,jsonencode(s));fclose(fid);
fid=fopen(fullfile(folder,'scene_handles.json'),'w');fwrite(fid,jsonencode(info));fclose(fid);
fprintf('Fresh OctoAssist scene built: %d hand DOF plus 6 arm DOF.\n',s.handDOF);
add_irregular_object();add_extreme_object();setup_three_objects();
end
