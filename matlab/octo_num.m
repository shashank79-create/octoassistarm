function v=octo_num(x)
if iscell(x),v=[];for k=1:numel(x),v=[v octo_num(x{k})];end;else,v=double(x(:)).';end
end
