local sim=require('sim');local json=require('dkjson')
function prepareSceneSupports(handles,selected)
 local previous=sim.setStepping(true)
 local ok,value=xpcall(function()
  local H=json.decode(handles);local heights={sphere=.025,cylinder=.04,box=.0225};if H.objects.irregular then local g=json.decode(sim.readCustomStringData(H.root,'OctoIrregularGeometry'));heights.irregular=g.halfHeight end;if H.objects.extreme then local g=json.decode(sim.readCustomStringData(H.root,'OctoExtremeGeometry'));heights.extreme=g.halfHeight end;local result={}
  for kind in pairs(H.objects)do
   local alias='SceneSupport_'..kind;local plate=-1
   for _,h in ipairs(sim.getObjectsInTree(sim.handle_scene,sim.object_shape_type,0))do if sim.getObjectAlias(h)==alias then plate=h;break end end
   if plate<0 then plate=sim.createPrimitiveShape(sim.primitiveshape_cuboid,{.12,.12,.006},0);sim.setObjectAlias(plate,alias)end
   local p=sim.getObjectPosition(H.objects[kind],sim.handle_world)
   sim.setObjectPosition(plate,{p[1],p[2],p[3]-heights[kind]-.003},sim.handle_world)
   local enabled=kind~=selected and p[3]-heights[kind]>.001
   sim.setObjectInt32Param(plate,sim.shapeintparam_static,1)
   sim.setObjectInt32Param(plate,sim.shapeintparam_respondable,enabled and 1 or 0)
   sim.setObjectInt32Param(plate,sim.objintparam_visibility_layer,enabled and 1 or 0)
   sim.setObjectSpecialProperty(plate,sim.objectspecialproperty_collidable+sim.objectspecialproperty_measurable+sim.objectspecialproperty_renderable)
   sim.setShapeColor(plate,nil,sim.colorcomponent_ambient_diffuse,{.46,.53,.57})
   result[#result+1]=plate
  end
  return result
 end,debug.traceback)
 sim.setStepping(previous);assert(ok,value);return value
end
