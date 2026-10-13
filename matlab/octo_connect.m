function [sim,client]=octo_connect()
api='C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\matlab';
assert(isfolder(api),'Set api to your CoppeliaSim ZeroMQ MATLAB client directory.');addpath(api);
jar=fullfile(api,'jeromq-0.5.2.jar');if ~any(strcmp(javaclasspath('-dynamic'),jar)),javaaddpath(jar);end
client=RemoteAPIClient('host','localhost','port',23000);sim=client.require('sim');
end
