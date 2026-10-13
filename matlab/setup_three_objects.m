function setup_three_objects()
% Explicit one-time staging. Picking commands subsequently read actual poses.
folder=fileparts(fileparts(mfilename('fullpath')));[sim,client]=octo_connect(); %#ok<ASGLU>
octo_prepare_stopped(sim);
names={'sphere','cylinder','box','irregular','extreme'};aliases={'TaskSphere','TaskCylinder','TaskBox','TaskIrregular','TaskExtreme'};
% World-frame compass layout around the arm base at (0,0).
positions=[0 .42 .025;0 -.42 .06;.42 0 .06;-.40 0 .06;-.30 .30 .06];
root=octo_num(sim.getObject('/OctoAssist'));h=jsondecode(sim.readCustomStringData(root,'OctoHandles'));
resolver=octo_num(sim.createScript(sim.scripttype_customization,fileread(fullfile(folder,'runtime','resolve_handles.lua')),0,'lua'));sim.initScript(resolver);
h=sim.callScriptFunction('resolveForMatlab',resolver,jsonencode(octo_config()),jsonencode(h));sim.removeObjects({resolver});
for k=1:numel(names)
 object=octo_num(sim.getObject(['/' aliases{k}]));h.objects.(names{k})=object;
 sim.setObjectPosition(object,positions(k,:),sim.handle_world);sim.setObjectQuaternion(object,[0 0 0 1],sim.handle_world);
 sim.setVector3Property(object,'initLinearVelocity',[0 0 0]);sim.setVector3Property(object,'initAngularVelocity',[0 0 0]);
 sim.setBoolProperty(object,'startInDynSleepMode',true);sim.resetDynamicObject(object);
 fprintf('%s at [%g %g %g] m\n',names{k},positions(k,:));
end
octo_scene_supports(sim,h,'');
supports=octo_num(h.supports);
for k=1:2
 sim.setObjectInt32Param(supports(k),sim.shapeintparam_respondable,0);sim.setObjectInt32Param(supports(k),sim.objintparam_visibility_layer,0);
end
s=octo_config();home=octo_ik(s.homePalm,s);arm=octo_num(h.arm);
for k=1:6,sim.setJointPosition(arm(k),home(k));end
sim.writeCustomStringData(root,'OctoHandles',jsonencode(h));
sim.writeCustomStringData(root,'OctoJob','null');
sim.setStringProperty(sim.getObject('/OctoAssist/OctoAssistRuntime'),'code',octo_runtime_code(folder));
reset_view();
sim.saveScene(fullfile(folder,'scene','OctoAssist.ttt'));
fprintf('All five objects staged. Run run_octopus and select the existing object.\n');
end
