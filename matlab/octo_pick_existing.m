function result=octo_pick_existing(kind,destination,options)
% Locate an existing named scene object; never spawn or reposition a payload.
if nargin<3,options=struct();end
kind=lower(char(kind));assert(any(strcmp(kind,{'sphere','cylinder','box','irregular','extreme'})),'Select sphere, cylinder, box, irregular, or extreme.');
options.pickExisting=true;
result=octo_pick_place([],destination,kind,options);
end
