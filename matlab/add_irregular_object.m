function add_irregular_object()
folder=fileparts(fileparts(mfilename('fullpath')));[sim,client]=octo_connect(); %#ok<ASGLU>
octo_preserve_objects_on_stop(sim);
script=octo_num(sim.createScript(sim.scripttype_customization,fileread(fullfile(folder,'runtime','add_irregular.lua')),0,'lua'));
cleanup=onCleanup(@()sim.removeObjects({script}));sim.initScript(script);
g=sim.callScriptFunction('addIrregular',script,fileread(fullfile(folder,'runtime','irregular_mesh.json')));
fid=fopen(fullfile(folder,'irregular_geometry.json'),'w');fwrite(fid,jsonencode(g));fclose(fid);clear cleanup
root=sim.getObject('/OctoAssist');h=jsondecode(sim.readCustomStringData(root,'OctoHandles'));
h.objects.irregular=octo_num(sim.getObject('/TaskIrregular'));sim.writeCustomStringData(root,'OctoHandles',jsonencode(h));
fid=fopen(fullfile(folder,'scene_handles.json'),'w');fwrite(fid,jsonencode(h));fclose(fid);
octo_scene_supports(sim,h,'');sim.setStringProperty(sim.getObject('/OctoAssist/OctoAssistRuntime'),'code',octo_runtime_code(folder));
sim.writeCustomStringData(root,'OctoSpec',jsonencode(octo_config()));sim.writeCustomStringData(root,'OctoJob','null');
reset_view();sim.saveScene(fullfile(folder,'scene','OctoAssist.ttt'));
fprintf('Added independent 100 g irregular pebble west of the base at [-0.40,0,0.06].\n');
end
