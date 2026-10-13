function octo_cleanup_preflight(sim,script,displayWasEnabled)
% Remove invisible preflight geometry if interrupted or an RPC fails.
try,sim.callScriptFunction('cancelPaths',script);catch,end
try,sim.removeObjects({script});catch,end
if nargin>=3
 try,sim.setBoolParam(sim.boolparam_display_enabled,displayWasEnabled);catch,end
end
end
