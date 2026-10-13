function s=octo_config()
% Fresh architecture: five-section digits with fixed vacuum cups.
% SI units, world +Z upward. Object remains a world-level dynamic shape.
s.names={'Index','Middle','Ring','Little'};s.R=.025;s.radius=.00325;
s.palm=[.063 .044 .014];s.H=.058;s.phi=[125 145 165 185 205];
xs=[-.0195 -.0065 .0065 .0195];s.fingers=cell(1,4);
for f=1:4
 h=sqrt((s.R+s.radius)^2-xs(f)^2);a=s.phi*pi/180;
 yz=[(h-sin(a(1))*s.H)/cos(a(1)),s.H];
 for k=1:4,yz(end+1,:)=tangent(h,s.phi(k),s.phi(k+1));end
 yz(end+1,:)=tangent(h,s.phi(end),s.phi(end)+15);
 points=[repmat(xs(f),6,1) yz];L=vecnorm(diff(points),2,2)';
 cups=zeros(5,3);normals=zeros(5,3);
 for k=1:5
  center=[xs(f),h*cos(a(k)),h*sin(a(k))];n=-center/(s.R+s.radius);
  cups(k,:)=center+s.radius*n;normals(k,:)=n;
 end
 s.fingers{f}=struct('rootS',points(1,:),'pointsS',points,'lengths',L,'cupPointsS',cups,'cupNormalsS',normals,'rest',zeros(1,5),'nominal',[35 20 20 20 20]*pi/180);
end
front=max(cellfun(@(g)g.rootS(2),s.fingers))+.00675;
s.palmCenterS=[0 front+s.palm(2)/2 s.H];s.objectP=-s.palmCenterS;
for f=1:4,g=s.fingers{f};g.rootP=g.rootS-s.palmCenterS;s.fingers{f}=g;end
h=s.R+s.radius;angles=[45 10 -25];yz=[(h-sin(pi/4)*.032)/cos(pi/4),.032];
for k=1:2,yz(end+1,:)=tangent(h,angles(k),angles(k+1));end
yz(end+1,:)=tangent(h,-25,-40);points=[zeros(4,1) yz];cmcS=[-.034 .025 .022];
v=points(1,:)-cmcS;az=atan2(v(2),v(1));el=atan2(v(3),hypot(v(1),v(2)));
cups=zeros(3,3);normals=zeros(3,3);
for k=1:3,c=[0 h*cosd(angles(k)) h*sind(angles(k))];n=-c/h;cups(k,:)=c+s.radius*n;normals(k,:)=n;end
s.thumb=struct('cmcP',cmcS-s.palmCenterS,'rootP',points(1,:)-s.palmCenterS,'metaLength',norm(v),'angles',[az el],'pointsS',points,'lengths',vecnorm(diff(points),2,2)','cupPointsS',cups,'cupNormalsS',normals,'rest',[0 0 0],'nominal',[-45 -35 -35]*pi/180);
s.handDOF=29;s.arm=struct('shoulderZ',.55,'upper',.35,'forearm',.32,'wristDrop',.06,'toolDrop',.035,'speed',.35);
s.homePalm=[.40 0 .35];s.primarySource=[.42 -.08 .025];s.primaryDestination=[.35 .18 .025];
s.vacuum=struct('pressurePa',20000,'cupRadius',.003,'stiffness',60,'damping',.8,'friction',.6,'sealGap',.0015,'sealNormalDegrees',45,'sealStretch',.006,'sealShear',.006,'rampSeconds',.35,'minSeals',4);
s.vacuum.contactStiffness=120;s.vacuum.contactDamping=.4;s.vacuum.contactForceCap=1.0;
s.arm.limits=[pi repmat(155*pi/180,1,4) pi];
s.extreme=struct('halfHeight',.019,'components',[]);
extremeFile=fullfile(fileparts(fileparts(mfilename('fullpath'))),'extreme_geometry.json');
if isfile(extremeFile),s.extreme=jsondecode(fileread(extremeFile));end
s.irregular=struct('halfHeight',.027,'planes',[]);
meshFile=fullfile(fileparts(fileparts(mfilename('fullpath'))),'irregular_geometry.json');
if isfile(meshFile),s.irregular=jsondecode(fileread(meshFile));end
s.dt=.005;s.mass=.1;s.groundMargin=.002;s.contactGap=.00025;
s.rollingResistance=.02;
end
function p=tangent(h,a,b),p=h/cosd((b-a)/2)*[cosd((a+b)/2),sind((a+b)/2)];end
