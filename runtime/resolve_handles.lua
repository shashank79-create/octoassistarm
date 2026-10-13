-- Bind by unique aliases after reload; saved numeric handles are informational.
function resolveGeometryHandles(S,H)
 local sim=require('sim');local aliases={};local counts={}
 for _,h in ipairs(sim.getObjectsInTree(sim.handle_scene,sim.handle_all,0))do local n=sim.getObjectAlias(h);aliases[n]=h;counts[n]=(counts[n]or 0)+1 end
 local function find(n)assert(counts[n]==1,'Missing or duplicate scene alias: '..n);return aliases[n]end
 H.root=find('OctoAssist');H.palm=find('HumanPalm')
 for k=1,6 do H.arm[k]=find('Arm_J'..k)end
 for f,g in ipairs(H.digits)do
  local n=f==5 and 'Thumb' or S.names[f]
  for k=1,#g.joints do g.joints[k]=find(n..'_Flex'..k);g.links[k]=find(n..'_RoundedModule'..k);g.cups[k]=find(n..'_SuctionCup'..k)end
  if f<5 then g.splay=find(n..'_Splay')end
 end
 H.thumbYaw=find('Thumb_CMC_Yaw');H.thumbElevation=find('Thumb_CMC_Elevation')
 for _,c in ipairs(H.cups)do local g=H.digits[c.digit];c.handle=g.cups[c.segment];c.link=g.links[c.segment]end
 H.floor=find('Floor');H.supports={find('SourceSupport'),find('DestinationSupport')}
 for _,kind in ipairs({'sphere','cylinder','box','irregular','extreme'})do local plate=aliases['SceneSupport_'..kind];if plate and sim.getObjectInt32Param(plate,sim.shapeintparam_respondable)==1 then H.supports[#H.supports+1]=plate end end
 H.objects={sphere=find('TaskSphere'),cylinder=find('TaskCylinder'),box=find('TaskBox')};if aliases.TaskIrregular then H.objects.irregular=find('TaskIrregular')end;if aliases.TaskExtreme then H.objects.extreme=find('TaskExtreme')end;return H
end
function resolveForMatlab(specJSON,handlesJSON)
 local previous=require('sim').setStepping(true)
 local ok,value=xpcall(function()
  local json=require('dkjson');return resolveGeometryHandles(json.decode(specJSON),json.decode(handlesJSON))
 end,debug.traceback)
 require('sim').setStepping(previous)
 assert(ok,value);return value
end
