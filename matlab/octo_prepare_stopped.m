function octo_prepare_stopped(sim)
% Completed tasks leave their final pose visible until the next command.
stopped=octo_num(sim.simulation_stopped);
if octo_num(sim.getSimulationState())==stopped,return;end
try
 runtime=octo_num(sim.getObject('/OctoAssist/OctoAssistRuntime'));
 status=sim.callScriptFunction('getStatus',runtime);
catch
 error('OctoAssist:RunningScene','CoppeliaSim is running. Click Stop before preparing this scene.');
end
finished=status.done || status.failed || strcmp(status.phase,'IDLE');
if ~finished
 error('OctoAssist:TaskInProgress', ...
  'The arm is currently in %s. Wait for DONE, or click Stop to interrupt this task.',status.phase);
end
fprintf('Stopping previous %s task before preparing the next command...\n',status.phase);
sim.stopSimulation();timer=tic;
while octo_num(sim.getSimulationState())~=stopped
 assert(toc(timer)<20,'CoppeliaSim has not stopped yet. Wait for Stop to finish, then retry.');
 pause(.05);
end
end
