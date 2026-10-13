local sim=require('sim');local json=require('dkjson')
function addExtreme()
 local previous=sim.setStepping(true)
 local ok,result=xpcall(function()
  local root=sim.getObject('/OctoAssist');local h=-1
  for _,v in ipairs(sim.getObjectsInTree(sim.handle_scene,sim.object_shape_type,0))do if sim.getObjectAlias(v)=='TaskExtreme' then h=v end end
  -- Abutting boxes: disjoint interiors, joined as one compound rigid body.
  -- Recesses remain empty in rendering, distance queries and Bullet collision.
  local parts={
   {center={0,0,0},half={.015,.015,.019}},
   {center={.025,0,.007},half={.010,.008,.012}},
   {center={-.023,.003,-.005},half={.008,.010,.014}},
   {center={.004,.025,-.003},half={.008,.010,.016}},
   {center={-.004,-.022,.006},half={.010,.007,.013}},
   {center={.004,.003,.024},half={.007,.008,.005}}
  }
  if h<0 then
   local hs={};local volume=0
   for _,b in ipairs(parts)do volume=volume+8*b.half[1]*b.half[2]*b.half[3]end
   for _,b in ipairs(parts)do
    local v=sim.createPrimitiveShape(sim.primitiveshape_cuboid,{2*b.half[1],2*b.half[2],2*b.half[3]},0)
    sim.setObjectPosition(v,b.center,sim.handle_world)
    assert(sim.computeMassAndInertia(v,.1/volume)==1)
    sim.setShapeColor(v,nil,sim.colorcomponent_ambient_diffuse,{.85,.22,.12});hs[#hs+1]=v
   end
   h=sim.groupShapes(hs,false);sim.setObjectAlias(h,'TaskExtreme')
   -- Rebase the reference frame on the design origin without moving its geometry.

   local rebased=sim.relocateShapeFrame(h,{0,0,0,0,0,0,1});assert(rebased==true or rebased==1,'Cannot rebase compound frame')
   sim.setObjectInt32Param(h,sim.shapeintparam_static,0)
   sim.setObjectInt32Param(h,sim.shapeintparam_respondable,1)
   sim.setObjectInt32Param(h,sim.shapeintparam_respondable_mask,65280)
   sim.setObjectSpecialProperty(h,sim.objectspecialproperty_collidable+sim.objectspecialproperty_measurable+sim.objectspecialproperty_renderable)
   sim.setObjectPosition(h,{-.30,.30,.06},sim.handle_world)
   sim.setBoolProperty(h,'startInDynSleepMode',true)
  end
  assert(math.abs(sim.getShapeMass(h)-.1)<1e-8,'Compound mass must be 100 g')
  assert(sim.getObjectParent(h)==-1,'Payload must be independent')
  local geometry={halfHeight=.019,components=parts,mass=.1,description='Six abutting convex boxes forming an asymmetric concave stepped solid; no convex hull replacement'}
  sim.writeCustomStringData(root,'OctoExtremeGeometry',json.encode(geometry));return geometry
 end,debug.traceback)
 sim.setStepping(previous);assert(ok,result);return result
end
