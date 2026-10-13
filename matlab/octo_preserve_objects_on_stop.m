function octo_preserve_objects_on_stop(sim)
% Keep actual payload locations when CoppeliaSim restores its initial scene.
if octo_num(sim.getSimulationState())==octo_num(sim.simulation_stopped),return;end
names={'TaskSphere','TaskCylinder','TaskBox','TaskIrregular','TaskExtreme'};poses=cell(1,numel(names));handles=zeros(1,numel(names));
for k=1:numel(names)
 try,handles(k)=octo_num(sim.getObject(['/' names{k}]));catch,continue;end
 poses{k}=octo_num(sim.getObjectPose(handles(k),sim.handle_world));
end
octo_prepare_stopped(sim);
for k=1:numel(names)
 if handles(k)==0,continue;end
 sim.setObjectPosition(handles(k),poses{k}(1:3),sim.handle_world);sim.setObjectQuaternion(handles(k),poses{k}(4:7),sim.handle_world);sim.resetDynamicObject(handles(k));
 actual=octo_num(sim.getObjectPosition(handles(k),sim.handle_world));
 assert(norm(actual-poses{k}(1:3))<1e-12,'Failed to retain actual object position after stopping.');
end
end
