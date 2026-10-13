function reset_view()
% Frame every arm/hand shape and all payloads without stopping a task.
folder=fileparts(fileparts(mfilename('fullpath')));[sim,client]=octo_connect(); %#ok<ASGLU>
root=octo_num(sim.getObject('/OctoAssist'));
shapes=octo_num(sim.getObjectsInTree(root,sim.object_shape_type,0));
for name={'TaskSphere','TaskCylinder','TaskBox','TaskIrregular','TaskExtreme'}
 try,shapes(end+1)=octo_num(sim.getObject(['/' name{1}]));catch err
  if any(strcmp(name{1},{'TaskIrregular','TaskExtreme'})),continue;end
  rethrow(err);
 end
end
sim.setBoolParam(sim.boolparam_display_enabled,true);
ok=octo_num(sim.cameraFitToView(0,shapes,1,1));
assert(ok==1,'The main camera view is unavailable. Select the scene viewport and retry reset_view.');
if octo_num(sim.getSimulationState())==octo_num(sim.simulation_stopped)
 sim.saveScene(fullfile(folder,'scene','OctoAssist.ttt'));
end
fprintf('View reset: full arm, hand, and five objects framed.\n');
end
