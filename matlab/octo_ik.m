function q=octo_ik(palmPosition,s)
% Exact 6-axis solution for the default horizontal-palm orientation.
% Positive shoulder/elbow pitch about +Y bends the arm downward.
p=palmPosition(:)';a=s.arm;rho=hypot(p(1),p(2));z=p(3)+a.wristDrop+a.toolDrop-a.shoulderZ;
c=(rho^2+z^2-a.upper^2-a.forearm^2)/(2*a.upper*a.forearm);
assert(abs(c)<=1,'Requested palm position is outside this arm workspace.');
elbow=acos(max(-1,min(1,c)));shoulder=atan2(-z,rho)-atan2(a.forearm*sin(elbow),a.upper+a.forearm*cos(elbow));
yaw=atan2(p(2),p(1));q=[yaw shoulder elbow 0 -shoulder-elbow -yaw];
assert(all(abs(q)<s.arm.limits),'Required pose exceeds the joint planning limits.');
assert(p(3)>=s.palm(3)/2+.002,'Palm would intersect the floor.');
end
