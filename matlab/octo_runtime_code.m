function code=octo_runtime_code(folder)
if nargin<1,folder=fileparts(fileparts(mfilename('fullpath')));end
code=[fileread(fullfile(folder,'runtime','resolve_handles.lua')),newline,fileread(fullfile(folder,'runtime','octo_runtime.lua'))];
end
