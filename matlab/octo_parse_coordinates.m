function xyz=octo_parse_coordinates(text)
% Accept [x y z], x,y,z, or (x,y,z) as plain numeric text, without evaluation.
text=strtrim(char(text));
if ~isempty(text) && any(text(1)=='[(')
 closing=']';if text(1)=='(',closing=')';end
 assert(text(end)==closing,'Close the coordinate brackets. Example: [0.42 -0.08 0.025]');
 text=text(2:end-1);
end
assert(~isempty(regexp(text,'^[\s,;0-9eE+.\-]+$','once')), ...
 'Enter only three numbers in metres. Example: 0.42, -0.08, 0.025');
parts=regexp(strtrim(text),'[\s,;]+','split');xyz=str2double(parts);
assert(numel(xyz)==3 && all(isfinite(xyz)), ...
 'Enter exactly three finite numbers: x, y, z. Example: 0.42, -0.08, 0.025');
xyz=reshape(xyz,1,3);
end
