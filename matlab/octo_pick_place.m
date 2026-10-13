function result=octo_pick_place(source,destination,kind,options)
% Coordinates are object centers in the CoppeliaSim world frame, metres.
% Existing-object mode reads its pose; coordinate-demo mode stages the source.
% During simulation it is moved exclusively by contact, gravity and vacuum forces.
if nargin<3,kind='sphere';end
if nargin<4,options=struct();end
if ~isfield(options,'fastTest'),options.fastTest=false;end
if ~isfield(options,'pickExisting'),options.pickExisting=false;end
if options.pickExisting
 [sim,client]=octo_connect(); %#ok<ASGLU>
 octo_preserve_objects_on_stop(sim);
 aliases=struct('sphere','TaskSphere','cylinder','TaskCylinder','box','TaskBox','irregular','TaskIrregular','extreme','TaskExtreme');
 source=octo_num(sim.getObjectPosition(sim.getObject(['/' aliases.(kind)]),sim.handle_world));
 fprintf('Located existing %s at [%g %g %g] m.\n',kind,source);
end
validateattributes(source,{'numeric'},{'vector','numel',3,'finite'});validateattributes(destination,{'numeric'},{'vector','numel',3,'finite'});
source=source(:)';destination=destination(:)';assert(any(strcmp(kind,{'sphere','cylinder','box','irregular','extreme'})),'Supported starter objects: sphere, cylinder, box.');
cfg=octo_config();halfHeight=struct('sphere',.025,'cylinder',.040,'box',.0225,'irregular',cfg.irregular.halfHeight,'extreme',cfg.extreme.halfHeight);height=halfHeight.(kind);
sourceTolerance=0;if options.pickExisting,sourceTolerance=.001;end
assert(source(3)>=height-sourceTolerance && destination(3)>=height, ...
 '%s centre Z must be at least %.4f m to clear the floor. Source Z=%g, destination Z=%g.', ...
 kind,height,source(3),destination(3));
folder=fileparts(fileparts(mfilename('fullpath')));s=octo_config();
% Raise the open hand above this payload's unusually high support surface.
if strcmp(kind,'extreme'),s.objectP(3)=s.objectP(3)-.012;end
[sim,client]=octo_connect(); %#ok<ASGLU>
octo_prepare_stopped(sim);
root=octo_num(sim.getObject('/OctoAssist'));h=jsondecode(sim.readCustomStringData(root,'OctoHandles'));
resolver=octo_num(sim.createScript(sim.scripttype_customization,fileread(fullfile(folder,'runtime','resolve_handles.lua')),0,'lua'));sim.initScript(resolver);
h=sim.callScriptFunction('resolveForMatlab',resolver,jsonencode(s),jsonencode(h));sim.removeObjects({resolver});
sim.writeCustomStringData(root,'OctoHandles',jsonencode(h));
sim.writeCustomStringData(root,'OctoSpec',jsonencode(s));
fid=fopen(fullfile(folder,'geometry.json'),'w');fwrite(fid,jsonencode(s));fclose(fid);
object=h.objects.(kind);assert(octo_num(sim.getObjectParent(object))==-1,'Task object is attached to something; refuse to run.');
sim.setFloatProperty(object,'mass',s.mass);if ~any(strcmp(kind,{'irregular','extreme'})),sim.setVector3Property(object,'centerOfMass',[0 0 0]);end
switch kind
 case 'sphere',inertias=repmat(.4*s.mass*.025^2,1,3);
 case 'cylinder',inertias=[repmat(s.mass*(3*.025^2+.08^2)/12,1,2),.5*s.mass*.025^2];
 case 'box',inertias=repmat(s.mass*.045^2/6,1,3);
 case {'irregular','extreme'},[originalInertia,originalCom]=sim.getShapeInertia(object);inertiaExpected=octo_num(originalInertia);
end
if ~any(strcmp(kind,{'irregular','extreme'}))
 inertiaExpected=reshape(diag(inertias)',1,9);sim.setShapeInertia(object,inertiaExpected,[1 0 0 0 0 1 0 0 0 0 1 0]);
end
sim.setFloatParam(sim.floatparam_simulation_time_step,s.dt);
sim.setBoolProperty(sim.handle_scene,'realtimeSimulation',false);
sim.setIntProperty(sim.handle_scene,'stepsPerRendering',10);
sim.setIntProperty(sim.handle_scene,'speedModifier',0);
% Coordinate-demo mode parks inactive exemplars; existing-object mode preserves them.
names={'sphere','cylinder','box','irregular','extreme'};parking=[-.40,-.40,.025;-.40,.40,.040;-.60,0,.0225;-.60,-.40,.06;-.60,.60,.06];
for k=1:numel(names)
 if ~options.pickExisting && ~strcmp(names{k},kind)
  unused=h.objects.(names{k});sim.setObjectPosition(unused,parking(k,:),-1);
  sim.setObjectQuaternion(unused,[0 0 0 1],-1);sim.resetDynamicObject(unused);
 end
end
if options.pickExisting
 octo_scene_supports(sim,h,kind);
 resolver=octo_num(sim.createScript(sim.scripttype_customization,fileread(fullfile(folder,'runtime','resolve_handles.lua')),0,'lua'));sim.initScript(resolver);
 h=sim.callScriptFunction('resolveForMatlab',resolver,jsonencode(s),jsonencode(h));sim.removeObjects({resolver});
 sim.writeCustomStringData(root,'OctoHandles',jsonencode(h));
 for k=1:numel(names)
  if ~strcmp(names{k},kind)
   other=octo_num(sim.getObjectPosition(h.objects.(names{k}),sim.handle_world));
   assert(norm(destination(1:2)-other(1:2))>.12,'Destination is too close to another object. Choose at least 0.12 m horizontal separation.');
  end
 end
end
goalPalm=source-s.objectP;placePalm=destination-s.objectP;
positions=[goalPalm+[0 0 .08];goalPalm;goalPalm+[0 0 .10];placePalm+[0 0 .10];placePalm+[0 0 .001];placePalm+[0 0 .12]];
paths=cell(1,6);previous=octo_ik(s.homePalm,s);
for k=1:6
 q=octo_ik(positions(k,:),s);duration=max(1.5,1.875*max(abs(q-previous))/s.arm.speed);
 paths{k}=struct('q',q,'duration',duration);previous=q;
end
% Physical support plates let raised source/destination coordinates remain supported.
supportHandles=octo_num(h.supports);
for k=1:2
 p=source;if k==2,p=destination;end
 sim.setObjectPosition(supportHandles(k),p+[0 0 -height-.003],-1);
 % At floor level, use the floor alone instead of two coplanar collision surfaces.
 sim.setObjectInt32Param(supportHandles(k),sim.shapeintparam_respondable,double(p(3)-height>1e-6));
 sim.setObjectInt32Param(supportHandles(k),sim.objintparam_visibility_layer,double(p(3)-height>1e-6));
end
quat=[0 0 0 1];
if ~options.pickExisting
 sim.setObjectPosition(object,source,-1);sim.setObjectQuaternion(object,quat,-1);
end
sim.resetDynamicObject(object);
sim.setVector3Property(object,'initLinearVelocity',[0 0 0]);sim.setVector3Property(object,'initAngularVelocity',[0 0 0]);
sim.setBoolProperty(object,'startInDynSleepMode',true);sim.resetDynamicObject(object);
arm=octo_num(h.arm);home=octo_ik(s.homePalm,s);for k=1:6,sim.setJointPosition(arm(k),home(k));end
for f=1:5
 if iscell(h.digits),digit=h.digits{f};else,digit=h.digits(f);end
 js=octo_num(digit.joints);if f==5,rest=s.thumb.rest;else,rest=s.fingers{f}.rest;end
 for k=1:numel(js),sim.setJointPosition(js(k),rest(k));end
end
yaw=s.thumb.angles(1);elevation=s.thumb.angles(2);
if strcmp(kind,'box'),elevation=elevation+.35;end
if strcmp(kind,'cylinder'),yaw=.10;elevation=elevation+.399;end
sim.setJointPosition(h.thumbYaw,yaw);
sim.setJointPosition(h.thumbElevation,elevation);
preflightScript=octo_num(sim.createScript(sim.scripttype_customization,fileread(fullfile(folder,'runtime','preflight.lua')),0,'lua'));
preflightCleanup=onCleanup(@()octo_cleanup_preflight(sim,preflightScript));
sim.initScript(preflightScript);
fprintf('Checking clearance on an invisible copy. The visible arm remains stationary.\n');
preflight=sim.callScriptFunction('beginPaths',preflightScript,jsonencode(s),jsonencode(h),jsonencode(paths),object);
while ~preflight.done
 preflight=sim.callScriptFunction('nextPathSample',preflightScript);
 if mod(preflight.samples,21)==0 && preflight.samples>0
  fprintf('Preflight clearance checks: %d / 126\n',preflight.samples);
 end
 if ~preflight.done,pause(.01);end % Let CoppeliaSim service its UI between RPCs.
end
clear preflightCleanup
assert(preflight.passed,preflight.reason);
fprintf('Preflight: %d samples; minimum support clearance %.3f mm.\n',preflight.samples,preflight.minimumGround*1000);
job=struct('source',source,'destination',destination,'kind',kind,'objectHandle',object,'paths',{paths});
if isfield(options,'recordPath')
 if ~exist(options.recordPath,'dir'),mkdir(options.recordPath);end
 job.recordPath=strrep(options.recordPath,'\','/');
end
sim.writeCustomStringData(root,'OctoJob',jsonencode(job));sim.saveScene(fullfile(folder,'scene','OctoAssist.ttt'));
runtime=octo_num(sim.getObject('/OctoAssist/OctoAssistRuntime'));
sim.setStringProperty(runtime,'code',octo_runtime_code(folder));
sim.saveScene(fullfile(folder,'scene','OctoAssist.ttt'));
[inertiaRead,comRead]=sim.getShapeInertia(object);
massProperties=struct('massKg',octo_num(sim.getShapeMass(object)),'inertiaKgM2',octo_num(inertiaRead),'centerOfMassFrame',octo_num(comRead));
assert(abs(massProperties.massKg-s.mass)<1e-10 && max(abs(massProperties.inertiaKgM2-inertiaExpected))<1e-10,'Physical mass/inertia readback mismatch.');
if isfield(options,'prepareOnly') && options.prepareOnly
 result=struct('prepared',true,'preflight',preflight,'job',job);return;
end
if options.fastTest
 previousDisplay=sim.getBoolParam(sim.boolparam_display_enabled);
 displayCleanup=onCleanup(@()sim.setBoolParam(sim.boolparam_display_enabled,previousDisplay));
 sim.setBoolParam(sim.boolparam_display_enabled,false);
else
 sim.setBoolParam(sim.boolparam_display_enabled,true);
end
fprintf('Clearance checks passed. Starting the physical grasp and carry.\n');
sim.startSimulation();wall=tic;lastPhase='';lastProgress=0;
while toc(wall)<900
 pause(.1);
 status=sim.callScriptFunction('getStatus',runtime);
 if ~strcmp(status.phase,lastPhase)
  fprintf('%s [%s]; sealed cups=%d\n',octo_phase_label(status.phase),status.phase,status.seals);lastPhase=status.phase;
 end
 if toc(wall)-lastProgress>=5 && isfield(status,'objectPosition')
  p=octo_num(status.objectPosition);
  fprintf('  [%s, sim %.1f s] Object [%0.4f %0.4f %0.4f] m; peak lift %.1f mm; cups=%d\n', ...
   status.phase,status.time,p,status.maxLift*1000,status.seals);
  lastProgress=toc(wall);
 end
 if status.done || status.failed,break;end
end
result=sim.callScriptFunction('getLog',runtime);result.wallSeconds=toc(wall);
if ~result.status.done && ~result.status.failed,result.status.failed=true;result.status.reason='Wall-clock deadline exceeded.';end
result.preflight=preflight;
result.massProperties=massProperties;
result.simulationSettings=struct('timeStep',octo_num(sim.getSimulationTimeStep()),'stepsPerRendering',octo_num(sim.getIntProperty(sim.handle_scene,'stepsPerRendering')));
result.status.success=result.status.done && ~result.status.failed && result.status.placeError<=.01 && result.status.maxHeight>source(3)+.05 && result.status.parent==-1 && result.status.seals==0 && norm(octo_num(result.status.finalVelocity))<.01;
if ~result.status.success && ~result.status.failed
 result.status.reason=sprintf('Finished but validation limits were not met: placement error %.1f mm (limit 10 mm), peak lift %.1f mm (must exceed 50 mm).',result.status.placeError*1000,(result.status.maxHeight-source(3))*1000);
end
fid=fopen(fullfile(folder,'logs',['latest_' kind '.json']),'w');fwrite(fid,jsonencode(result));fclose(fid);
fprintf('Completed=%d, success=%d, sealed cups max=%d\n',result.status.done,result.status.success,result.status.maxSeals);
if result.status.success
 fprintf('Placed object at [%0.4f %0.4f %0.4f] m; target error %.2f mm.\n',octo_num(result.status.finalObject),result.status.placeError*1000);
else
 fprintf(2,'Task stopped or did not meet validation limits: %s\n',result.status.reason);
end
if result.status.failed,sim.stopSimulation();end
% Leave a successful final state visible; simulation has no more commanded motion.
% The next project command stops a completed task. Stopping restores initial poses.
end
