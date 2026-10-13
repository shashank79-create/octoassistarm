local sim=require('sim');local json=require('dkjson')
function addIrregular(meshJSON)
 local previous=sim.setStepping(true)
 local ok,result=xpcall(function()
  local root=sim.getObject('/OctoAssist');local existing=-1
  for _,h in ipairs(sim.getObjectsInTree(sim.handle_scene,sim.object_shape_type,0))do if sim.getObjectAlias(h)=='TaskIrregular' then existing=h end end
  local h=existing
  if h<0 then
   local m=json.decode(meshJSON);h=sim.createMeshShape(1,math.pi/5,m.vertices,m.indices)
   sim.setObjectAlias(h,'TaskIrregular');sim.setObjectInt32Param(h,sim.shapeintparam_static,0)
   sim.setObjectInt32Param(h,sim.shapeintparam_respondable,1);sim.setObjectInt32Param(h,sim.shapeintparam_respondable_mask,65280)
   sim.setObjectSpecialProperty(h,sim.objectspecialproperty_collidable+sim.objectspecialproperty_measurable+sim.objectspecialproperty_renderable)
   sim.setShapeColor(h,nil,sim.colorcomponent_ambient_diffuse,{.20,.75,.35})
   assert(sim.computeMassAndInertia(h,1000)==1,'Irregular mesh is not convex')
   local mass=sim.getShapeMass(h);assert(sim.computeMassAndInertia(h,1000*.1/mass)==1)
   sim.setObjectPosition(h,{-.40,0,.06},sim.handle_world);sim.setObjectQuaternion(h,{0,0,0,1},sim.handle_world)
  end
  local vertices,indices=sim.getShapeMesh(h);local planes={};local minZ=math.huge
  for i=3,#vertices,3 do minZ=math.min(minZ,vertices[i])end
  for i=1,#indices,3 do
   local a,b,c={},{},{}
   for k=1,3 do a[k]=vertices[3*indices[i]+k];b[k]=vertices[3*indices[i+1]+k];c[k]=vertices[3*indices[i+2]+k]end
   local u,v={b[1]-a[1],b[2]-a[2],b[3]-a[3]},{c[1]-a[1],c[2]-a[2],c[3]-a[3]}
   local n={u[2]*v[3]-u[3]*v[2],u[3]*v[1]-u[1]*v[3],u[1]*v[2]-u[2]*v[1]};local length=math.sqrt(n[1]^2+n[2]^2+n[3]^2)
   for k=1,3 do n[k]=n[k]/length end
   local d=n[1]*a[1]+n[2]*a[2]+n[3]*a[3]
   -- Mesh winding is outward; verify its convex half-space against all vertices.
   local bad=0;for j=1,#vertices,3 do if n[1]*vertices[j]+n[2]*vertices[j+1]+n[3]*vertices[j+2]>d+1e-7 then bad=bad+1 end end
   assert(bad==0,'Mesh normal is not an outward convex support plane')
   planes[#planes+1]={n[1],n[2],n[3],d}
  end
  for _,name in ipairs({'Arm_J1','Arm_J6'})do sim.setJointInterval(sim.getObject('/'..name),false,{-math.pi,2*math.pi})end
  local geometry={halfHeight=-minZ,planes=planes,description='asymmetric convex pebble mesh',mass=.1}
  sim.writeCustomStringData(root,'OctoIrregularGeometry',json.encode(geometry));return geometry
 end,debug.traceback)
 sim.setStepping(previous);assert(ok,result);return result
end
