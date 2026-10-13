function octo_scene_supports(sim,h,selected)
folder=fileparts(fileparts(mfilename('fullpath')));
script=octo_num(sim.createScript(sim.scripttype_customization,fileread(fullfile(folder,'runtime','scene_supports.lua')),0,'lua'));
cleanup=onCleanup(@()sim.removeObjects({script}));sim.initScript(script);
sim.callScriptFunction('prepareSceneSupports',script,jsonencode(h),selected);
end
