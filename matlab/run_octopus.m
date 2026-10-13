function result=run_octopus(kind,source,destination,options)
% Set this object's start/end, or leave start blank to use its actual pose.
if nargin<4,options=struct();end
if nargin>0
 assert(nargin>=3,'Use run_octopus(kind,start,destination).');
 kind=lower(char(kind));
 if ~isempty(source),octo_set_start(kind,source,destination);end
 result=octo_pick_existing(kind,destination,options);return;
end
fprintf('Open scene/OctoAssist.ttt in CoppeliaSim. A completed task stops automatically before the next run.\n');
fprintf('Coordinates are object centres in metres. You can type x,y,z without brackets.\n');
fprintf('All five objects stay in the scene. Set this object''s start, or leave it blank to keep its actual location.\n');
kind=lower(strtrim(input('Object type: sphere / cylinder / box / irregular / extreme [sphere]: ','s')));
kind=strrep(strrep(kind,'''',''),'"','');
if isempty(kind),kind='sphere';end
assert(any(strcmp(kind,{'sphere','cylinder','box','irregular','extreme'})), ...
 'Object type must be sphere, cylinder, box, irregular, or extreme.');
exampleZ=.025;if ~strcmp(kind,'sphere'),exampleZ=.060;end
exampleY=.20;
exampleX=.35;
if any(strcmp(kind,{'irregular','extreme'}))
 exampleX=-.32;exampleY=.18;
end
if strcmp(kind,'cylinder')
 exampleY=-.20;
end
sourceText=input('Start position x,y,z [Enter = current scene position]: ','s');
source=[];if ~isempty(strtrim(sourceText)),source=octo_parse_coordinates(sourceText);end
destination=readCoordinates(sprintf('Placing position (example: %.2f,%.2f,%.3f): ',exampleX,exampleY,exampleZ));
fprintf('Picking the existing %s; its location is read from CoppeliaSim.\n',kind);
if ~isempty(source),octo_set_start(kind,source,destination);end
result=octo_pick_existing(kind,destination,options);
end
function xyz=readCoordinates(prompt)
while true
 text=input(prompt,'s');
 try,xyz=octo_parse_coordinates(text);return;
 catch err,fprintf('Input error: %s\nPlease enter the coordinates again.\n',err.message);end
end
end
