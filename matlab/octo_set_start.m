function octo_set_start(kind,source,destination)
% Stage only the selected existing payload while simulation is stopped.
names={'sphere','cylinder','box','irregular','extreme'};aliases={'TaskSphere','TaskCylinder','TaskBox','TaskIrregular','TaskExtreme'};
index=find(strcmp(kind,names));assert(~isempty(index),'Select sphere, cylinder, box, irregular, or extreme.');
validateattributes(source,{'numeric'},{'vector','numel',3,'finite'});
validateattributes(destination,{'numeric'},{'vector','numel',3,'finite'});
source=source(:)';destination=destination(:)';s=octo_config();heights=[.025 .040 .0225 s.irregular.halfHeight s.extreme.halfHeight];
assert(source(3)>=heights(index) && destination(3)>=heights(index),'Object centre is below its minimum floor height: %.4f m.',heights(index));
s=octo_config();
for p=[source;destination]'
 for dz=[0 .001 .08 .10 .12],octo_ik(p'-s.objectP+[0 0 dz],s);end
end
[sim,client]=octo_connect(); %#ok<ASGLU>
octo_preserve_objects_on_stop(sim);
for k=1:numel(names)
 if k~=index
  other=octo_num(sim.getObjectPosition(sim.getObject(['/' aliases{k}]),sim.handle_world));
  assert(norm(source(1:2)-other(1:2))>.12,'Start is too close to another object; use at least 0.12 m horizontal separation.');
  assert(norm(destination(1:2)-other(1:2))>.12,'Destination is too close to another object; use at least 0.12 m horizontal separation.');
 end
end
object=octo_num(sim.getObject(['/' aliases{index}]));
assert(octo_num(sim.getObjectParent(object))==-1,'Payload must be independent of the hand.');
sim.setObjectPosition(object,source,sim.handle_world);sim.setObjectQuaternion(object,[0 0 0 1],sim.handle_world);
sim.setVector3Property(object,'initLinearVelocity',[0 0 0]);sim.setVector3Property(object,'initAngularVelocity',[0 0 0]);
sim.setBoolProperty(object,'startInDynSleepMode',true);sim.resetDynamicObject(object);
folder=fileparts(fileparts(mfilename('fullpath')));root=octo_num(sim.getObject('/OctoAssist'));
h=jsondecode(sim.readCustomStringData(root,'OctoHandles'));
for k=1:numel(names),h.objects.(names{k})=octo_num(sim.getObject(['/' aliases{k}]));end
octo_scene_supports(sim,h,'');
% Clear the previous task's support plates; new task preparation rebuilds them.
for alias={'SourceSupport','DestinationSupport'}
 plate=sim.getObject(['/' alias{1}]);sim.setObjectInt32Param(plate,sim.shapeintparam_respondable,0);
 sim.setObjectInt32Param(plate,sim.objintparam_visibility_layer,0);
end
raw=sim.readCustomStringData(root,'OctoDestinations');goals=struct();
if (ischar(raw) || (isstring(raw) && isscalar(raw))) && ~isempty(raw)
 goals=jsondecode(raw);
end
goals.(kind)=destination;sim.writeCustomStringData(root,'OctoDestinations',jsonencode(goals));
sim.writeCustomStringData(root,'OctoJob','null');
sim.saveScene(fullfile(folder,'scene','OctoAssist.ttt'));
fprintf('%s staged at [%g %g %g]; destination [%g %g %g] m.\n',kind,source,destination);
end
