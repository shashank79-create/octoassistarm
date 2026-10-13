local sim=require('sim')
local json=require('dkjson')
local function add(a,b)return {a[1]+b[1],a[2]+b[2],a[3]+b[3]}end
local function sub(a,b)return {a[1]-b[1],a[2]-b[2],a[3]-b[3]}end
local function scale(a,k)return {a[1]*k,a[2]*k,a[3]*k}end
local function dot(a,b)return a[1]*b[1]+a[2]*b[2]+a[3]*b[3]end
local function cross(a,b)return {a[2]*b[3]-a[3]*b[2],a[3]*b[1]-a[1]*b[3],a[1]*b[2]-a[2]*b[1]}end
local function norm(a)return math.sqrt(dot(a,a))end
local function unit(a)return scale(a,1/norm(a))end
local I={1,0,0,0,1,0,0,0,1}
local X={0,0,1,0,1,0,-1,0,0}
local Y={1,0,0,0,0,1,0,-1,0}
local minusY={1,0,0,0,0,-1,0,1,0}
local function put(h,ref,p,R)
    sim.setObjectMatrix(h,{R[1],R[2],R[3],p[1],R[4],R[5],R[6],p[2],R[7],R[8],R[9],p[3]},ref)
end
local function orient(z)
    z=unit(z);local trial={0,0,1};if math.abs(dot(z,trial))>.9 then trial={1,0,0}end
    local x=unit(cross(trial,z));local y=cross(z,x)
    return {x[1],y[1],z[1],x[2],y[2],z[2],x[3],y[3],z[3]}
end
local function physical(h,color,respondable)
    sim.setObjectInt32Param(h,sim.shapeintparam_static,1)
    sim.setObjectInt32Param(h,sim.shapeintparam_respondable,respondable and 1 or 0)
    sim.setObjectInt32Param(h,sim.shapeintparam_respondable_mask,65280)
    sim.setObjectSpecialProperty(h,sim.objectspecialproperty_collidable+sim.objectspecialproperty_measurable+sim.objectspecialproperty_renderable)
    sim.setShapeColor(h,'@compound',sim.colorcomponent_ambient_diffuse,color)
end
local function capsule(parent,ref,a,b,r,name,color)
    local v=sub(b,a);local h=sim.createPrimitiveShape(sim.primitiveshape_capsule,{2*r,2*r,norm(v)+2*r},0)
    sim.setObjectAlias(h,name);sim.setObjectParent(h,parent,true);put(h,ref,scale(add(a,b),.5),orient(v));physical(h,color,true);return h
end
local function joint(parent,ref,p,R,name,lo,hi)
    local h=sim.createJoint(sim.joint_revolute,sim.jointmode_kinematic,0,{.004,.003})
    sim.setObjectAlias(h,name);sim.setObjectParent(h,parent,true);put(h,ref,p,R)
    sim.setJointInterval(h,false,{lo,hi-lo});sim.setJointPosition(h,0);sim.setObjectInt32Param(h,sim.objintparam_visibility_layer,0);return h
end
local function cup(parent,ref,p,n,name)
    -- Lip front at p. The tiny compliant cup is fixed to its host link.
    local h=sim.createPrimitiveShape(sim.primitiveshape_cylinder,{.006,.006,.0006},0)
    sim.setObjectAlias(h,name);sim.setObjectParent(h,parent,true);put(h,ref,sub(p,scale(n,.0003)),orient(n));physical(h,{.16,.26,.32},false)
    return h
end
local function taskObject(kind,sizes,pos,name,color,mass)
    local h=sim.createPrimitiveShape(kind,sizes,0);sim.setObjectAlias(h,name);put(h,-1,pos,I);physical(h,color,true)
    sim.setObjectInt32Param(h,sim.shapeintparam_static,0);sim.setShapeMass(h,mass)
    local xx,yy,zz
    if kind==sim.primitiveshape_spheroid then xx=.4*mass*(sizes[1]/2)^2;yy=xx;zz=xx
    elseif kind==sim.primitiveshape_cylinder then xx=mass*(3*(sizes[1]/2)^2+sizes[3]^2)/12;yy=xx;zz=.5*mass*(sizes[1]/2)^2
    else xx=mass*(sizes[2]^2+sizes[3]^2)/12;yy=mass*(sizes[1]^2+sizes[3]^2)/12;zz=mass*(sizes[1]^2+sizes[2]^2)/12 end
    sim.setShapeInertia(h,{xx,0,0,0,yy,0,0,0,zz},{1,0,0,0,0,1,0,0,0,0,1,0})
    return h
end
function buildFresh(specJSON)
    local s=json.decode(specJSON);local H={arm={},digits={},cups={},objects={}}
    H.root=sim.createPrimitiveShape(sim.primitiveshape_cylinder,{.12,.12,.10},0)
    sim.setObjectAlias(H.root,'OctoAssist');put(H.root,-1,{0,0,.05},I);physical(H.root,{.52,.58,.66},true);sim.setModelProperty(H.root,0)
    capsule(H.root,-1,{0,0,.1},{0,0,s.arm.shoulderZ},.045,'HumanArm_Pedestal',{.65,.69,.73})
    local shoulder={0,0,s.arm.shoulderZ};local elbow={s.arm.upper,0,s.arm.shoulderZ}
    local wrist={s.arm.upper+s.arm.forearm,0,s.arm.shoulderZ}
    H.arm[1]=joint(H.root,-1,shoulder,I,'Arm_J1',-math.pi,math.pi)
    H.arm[2]=joint(H.arm[1],-1,shoulder,Y,'Arm_J2',-2.7,2.7)
    local upper=capsule(H.arm[2],-1,shoulder,elbow,.028,'HumanArm_UpperArm',{.75,.78,.80})
    H.arm[3]=joint(upper,-1,elbow,Y,'Arm_J3',-2.7,2.7)
    local forearm=capsule(H.arm[3],-1,elbow,wrist,.023,'HumanArm_Forearm',{.75,.78,.80})
    H.arm[4]=joint(forearm,-1,wrist,X,'Arm_J4',-2.7,2.7)
    H.arm[5]=joint(H.arm[4],-1,wrist,Y,'Arm_J5',-2.7,2.7)
    local flange=add(wrist,{0,0,-s.arm.wristDrop})
    capsule(H.arm[5],-1,wrist,flange,.019,'HumanArm_Wrist',{.63,.68,.72})
    H.arm[6]=joint(H.arm[5],-1,flange,I,'Arm_J6',-math.pi,math.pi)
    H.palm=sim.createPrimitiveShape(sim.primitiveshape_cuboid,s.palm,0);sim.setObjectAlias(H.palm,'HumanPalm')
    sim.setObjectParent(H.palm,H.arm[6],true);put(H.palm,-1,add(flange,{0,0,-s.arm.toolDrop}),I);physical(H.palm,{.78,.81,.84},true)
    for f=1,4 do
        local g=s.fingers[f];local row={joints={},links={},cups={},active=1};local p=g.rootP
        capsule(H.palm,H.palm,{p[1],-s.palm[2]/2,0},p,.0025,s.names[f]..'_Metacarpal',{.7,.75,.78})
        row.splay=joint(H.palm,H.palm,p,I,s.names[f]..'_Splay',-.30,.30)
        local parent=row.splay
        for k=1,5 do
            row.joints[k]=joint(parent,H.palm,p,X,s.names[f]..'_Flex'..k,-.10,1.65)
            local ep=add(p,{0,-g.lengths[k],0})
            row.links[k]=capsule(row.joints[k],H.palm,p,ep,s.radius,s.names[f]..'_RoundedModule'..k,{.68,.75,.79})
            p=ep;parent=row.links[k]
        end
        for k=1,5 do sim.setJointPosition(row.joints[k],g.nominal[k])end
        for k=1,5 do
            local c=cup(row.links[k],H.palm,sub(g.cupPointsS[k],s.palmCenterS),g.cupNormalsS[k],s.names[f]..'_SuctionCup'..k)
            row.cups[k]=c;H.cups[#H.cups+1]={handle=c,link=row.links[k],digit=f,segment=k}
        end
        for k=1,5 do sim.setJointPosition(row.joints[k],g.rest[k])end
        H.digits[f]=row
    end
    local t=s.thumb;local row={joints={},links={},cups={},active=1}
    capsule(H.palm,H.palm,{-s.palm[1]/2,-s.palm[2]/2,0},t.cmcP,.0025,'Thumb_Thenar',{.70,.75,.78})
    H.thumbYaw=joint(H.palm,H.palm,t.cmcP,I,'Thumb_CMC_Yaw',t.angles[1]-.6,t.angles[1]+.6)
    H.thumbElevation=joint(H.thumbYaw,H.palm,t.cmcP,minusY,'Thumb_CMC_Elevation',t.angles[2]-.4,t.angles[2]+.4)
    sim.setJointPosition(H.thumbYaw,t.angles[1]);sim.setJointPosition(H.thumbElevation,t.angles[2])
    local meta=capsule(H.thumbElevation,H.palm,t.cmcP,t.rootP,.003,'Thumb_Metacarpal',{.70,.75,.78})
    local p=t.rootP;local parent=meta
    for k=1,3 do
        row.joints[k]=joint(parent,H.palm,p,X,'Thumb_Flex'..k,-1.6,.2)
        local ep=add(p,{0,t.lengths[k],0})
        row.links[k]=capsule(row.joints[k],H.palm,p,ep,s.radius,'Thumb_RoundedModule'..k,{.68,.75,.79});p=ep;parent=row.links[k]
    end
    for k=1,3 do sim.setJointPosition(row.joints[k],t.nominal[k])end
    for k=1,3 do
        local c=cup(row.links[k],H.palm,sub(t.cupPointsS[k],s.palmCenterS),t.cupNormalsS[k],'Thumb_SuctionCup'..k)
        row.cups[k]=c;H.cups[#H.cups+1]={handle=c,link=row.links[k],digit=5,segment=k}
    end
    for k=1,3 do sim.setJointPosition(row.joints[k],0)end
    H.digits[5]=row
    H.floor=sim.getObject('/Floor')
    H.supports={}
    for i,pos in ipairs({s.primarySource,s.primaryDestination})do
        local plate=sim.createPrimitiveShape(sim.primitiveshape_cuboid,{.14,.14,.006},0)
        sim.setObjectAlias(plate,i==1 and 'SourceSupport' or 'DestinationSupport');put(plate,-1,{pos[1],pos[2],pos[3]-s.R-.003},I);physical(plate,{.46,.53,.57},true);H.supports[i]=plate
    end
    H.objects.sphere=taskObject(sim.primitiveshape_spheroid,{.05,.05,.05},s.primarySource,'TaskSphere',{.93,.67,.20},s.mass)
    H.objects.cylinder=taskObject(sim.primitiveshape_cylinder,{.05,.05,.08},{-.25,.3,.04},'TaskCylinder',{.25,.60,.82},s.mass)
    H.objects.box=taskObject(sim.primitiveshape_cuboid,{.045,.045,.045},{-.3,-.2,.0225},'TaskBox',{.66,.36,.73},s.mass)
    for k=1,6 do sim.setJointPosition(H.arm[k],s.homeQ[k])end
    local cam=sim.getObject('/DefaultCamera');put(cam,-1,{.85,-.8,.72},orient(sub({.30,0,.27},{.85,-.8,.72})))
    sim.cameraFitToView(0,{H.root,H.objects.sphere,H.objects.cylinder,H.objects.box},0,1.15)
    return H
end
