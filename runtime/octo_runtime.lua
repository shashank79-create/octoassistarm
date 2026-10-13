-- Physics-enabled payload, ideal joint servos, contact-gated compliant vacuum.
-- Never sets the payload pose or parent during simulation.
local sim=require('sim')
local json=require('dkjson')
local function add(a,b)return {a[1]+b[1],a[2]+b[2],a[3]+b[3]}end
local function sub(a,b)return {a[1]-b[1],a[2]-b[2],a[3]-b[3]}end
local function mul(a,k)return {a[1]*k,a[2]*k,a[3]*k}end
local function dot(a,b)return a[1]*b[1]+a[2]*b[2]+a[3]*b[3]end
local function norm(a)return math.sqrt(dot(a,a))end
local function cross(a,b)return {a[2]*b[3]-a[3]*b[2],a[3]*b[1]-a[1]*b[3],a[1]*b[2]-a[2]*b[1]}end
local function clamp(x,a,b)return math.max(a,math.min(b,x))end
local function rotate(M,v)return {M[1]*v[1]+M[2]*v[2]+M[3]*v[3],M[5]*v[1]+M[6]*v[2]+M[7]*v[3],M[9]*v[1]+M[10]*v[2]+M[11]*v[3]}end
local function distance(a,b)local ok,d=sim.checkDistance(a,b,0);if ok==1 then return d[7],d end;return math.huge,nil end
local function event(kind,data)E[#E+1]={time=sim.getSimulationTime(),kind=kind,data=data}end
local function phase(name)
    state.phase=name;state.phaseStart=sim.getSimulationTime();event('phase',name)
end
local function fail(reason)state.failed=true;state.reason=reason;phase('FAILED')end
local function armQ()local q={};for i,h in ipairs(H.arm)do q[i]=sim.getJointPosition(h)end;return q end
local function setArm(q)for i,h in ipairs(H.arm)do sim.setJointPosition(h,q[i])end end
local function beginMove(index,nextPhase)
    state.moveIndex=index;state.moveFrom=armQ();state.nextPhase=nextPhase;phase('MOVE_'..index)
end
local function ik(p)
    local a=S.arm;local r=math.sqrt(p[1]^2+p[2]^2);local z=p[3]+a.wristDrop+a.toolDrop-a.shoulderZ
    local c=(r*r+z*z-a.upper*a.upper-a.forearm*a.forearm)/(2*a.upper*a.forearm)
    assert(math.abs(c)<=1,'Measured grasp requires an unreachable placement pose')
    local elbow=math.acos(clamp(c,-1,1));local shoulder=math.atan(-z,r)-math.atan(a.forearm*math.sin(elbow),a.upper+a.forearm*math.cos(elbow))
    local yaw=math.atan(p[2],p[1]);local q={yaw,shoulder,elbow,0,-shoulder-elbow,-yaw}
    for i,v in ipairs(q)do assert(math.abs(v)<S.arm.limits[i],'Measured grasp exceeds arm joint limits')end
    return q
end
local function adjustCarry()
    local offset=sub(sim.getObjectPosition(H.palm,sim.handle_world),sim.getObjectPosition(job.objectHandle,sim.handle_world))
    local p=add(job.destination,offset);local previous=armQ()
    for i=4,6 do
        local dz=i==4 and .10 or (i==5 and .001 or .12);local q=ik(add(p,{0,0,dz}));local delta=0
        for j=1,6 do delta=math.max(delta,math.abs(q[j]-previous[j]))end
        job.paths[i]={q=q,duration=math.max(1.5,1.875*delta/S.arm.speed)};previous=q
    end
    event('measured_grasp_offset',{offset=offset,placementPalm=p})
end
local function groundSafe()
    if distance(handCollection,H.floor)<S.groundMargin then return false end
    for _,support in ipairs(H.supports or {})do if distance(handCollection,support)<S.groundMargin then return false end end
    for _,other in pairs(H.objects)do if other~=job.objectHandle and distance(handCollection,other)<.002 then return false end end
    return true
end
local function digitGroundSafe(g,k)
    -- A flexion proposal moves only this digit's downstream links. The palm,
    -- arm and other digits retain their already-checked poses.
    for j=k,#g.links do
        if distance(g.links[j],H.floor)<S.groundMargin then return false end
        for _,h in ipairs(H.supports or {})do if distance(g.links[j],h)<S.groundMargin then return false end end
    end
    return true
end
local function objectNormal(p)
    local M=sim.getObjectMatrix(job.objectHandle,sim.handle_world)
    local inv=sim.getMatrixInverse(M);local pl=sim.multiplyVector(inv,p);local nl
    if job.kind=='sphere' then nl=mul(pl,1/math.max(norm(pl),1e-12))
    elseif job.kind=='cylinder' then
        if math.abs(pl[3])>.039 then nl={0,0,pl[3]>=0 and 1 or -1}
        else local rr=math.sqrt(pl[1]^2+pl[2]^2);nl={pl[1]/math.max(rr,1e-12),pl[2]/math.max(rr,1e-12),0}end
    elseif job.kind=='extreme' then
        -- Closest actual component face; no convex-hull normal for this concave body.
        local best=math.huge
        for _,b in ipairs(S.extreme.components)do
            local v=sub(pl,b.center);local outside={};local d=0
            for k=1,3 do outside[k]=v[k]-clamp(v[k],-b.half[k],b.half[k]);d=d+outside[k]^2 end
            local k=1
            if d>1e-16 then
                for j=2,3 do if math.abs(outside[j])>math.abs(outside[k])then k=j end end
            else
                local gap=math.huge
                for j=1,3 do if b.half[j]-math.abs(v[j])<gap then gap=b.half[j]-math.abs(v[j]);k=j end end
                d=gap*gap
            end
            if d<best then best=d;nl={0,0,0};nl[k]=v[k]>=0 and 1 or -1 end
        end
        assert(nl,'Extreme compound surface geometry is missing')
    elseif job.kind=='irregular' then
        local best=-math.huge;for _,plane in ipairs(S.irregular.planes)do local d=plane[1]*pl[1]+plane[2]*pl[2]+plane[3]*pl[3]-plane[4];if d>best then best=d;nl={plane[1],plane[2],plane[3]}end end
        assert(nl,'Irregular mesh surface planes are missing')
    else
        local k=1;for i=2,3 do if math.abs(pl[i])>math.abs(pl[k])then k=i end end
        nl={0,0,0};nl[k]=pl[k]>=0 and 1 or -1
    end
    return rotate(M,nl)
end
local function sealCups()
    if lastSealScan and sim.getSimulationTime()-lastSealScan<.20 then return end
    lastSealScan=sim.getSimulationTime()
    for i,c in ipairs(C)do
        if not c.anchor and not c.vented and H.digits[c.digit].active>c.segment then
            local cupGap,data=distance(c.handle,job.objectHandle)
            local linkGap=math.huge;if cupGap<=S.vacuum.sealGap then linkGap=distance(c.link,job.objectHandle)end
            if data and linkGap<=S.contactGap+.00015 and cupGap<=S.vacuum.sealGap then
                local M=sim.getObjectMatrix(c.handle,sim.handle_world)
                local cp=sim.multiplyVector(M,{0,0,.0003});local outward={-M[3],-M[7],-M[11]}
                local point={data[4],data[5],data[6]};local n=objectNormal(point)
                if dot(outward,n)>=math.cos(S.vacuum.sealNormalDegrees*math.pi/180)then
                    c.anchor=sim.multiplyVector(sim.getMatrixInverse(sim.getObjectMatrix(job.objectHandle,sim.handle_world)),point)
                    c.rest=rotate(sim.getMatrixInverse(M),sub(cp,point));c.sealTime=sim.getSimulationTime();c.prevCP=cp
                    event('seal',{cup=i,digit=c.digit,segment=c.segment,gap=cupGap})
                    return -- One newly enabled cup per scan: sequential vacuum engagement.
                end
            end
        end
    end
end
local function applyVacuum(dt)
    local Mobj=sim.getObjectMatrix(job.objectHandle,sim.handle_world)
    local objectP=sim.getObjectPosition(job.objectHandle,sim.handle_world)
    local v,w=sim.getVelocity(job.objectHandle);local F={0,0,0};local T={0,0,0};local count=0;local maxForce=0;local capacity=0
    for i,c in ipairs(C)do
        if c.anchor then
            local M=sim.getObjectMatrix(c.handle,sim.handle_world)
            local cp=sim.multiplyVector(M,{0,0,.0003});local n={-M[3],-M[7],-M[11]}
            local point=sim.multiplyVector(Mobj,c.anchor);local target=sub(cp,rotate(M,c.rest));local dx=sub(target,point)
            local dn=dot(dx,n);local tangent=sub(dx,mul(n,dn))
            if dn>S.vacuum.sealStretch or norm(tangent)>S.vacuum.sealShear then
                event('seal_break',{cup=i,normalExtension=dn,shear=norm(tangent)});c.anchor=nil;c.vented=true
            else
                local ramp=clamp((sim.getSimulationTime()-c.sealTime)/S.vacuum.rampSeconds,0,1)
                if c.ventStart then ramp=ramp*clamp(1-(sim.getSimulationTime()-c.ventStart)/S.vacuum.rampSeconds,0,1)end
                local cap=S.vacuum.pressurePa*math.pi*S.vacuum.cupRadius^2*ramp
                capacity=capacity+cap*(math.max(0,n[3])+S.vacuum.friction*math.sqrt(math.max(0,1-n[3]^2)))
                local cupV=mul(sub(cp,c.prevCP),1/math.max(dt,1e-6));c.prevCP=cp
                local relV=sub(cupV,add(v,cross(w,sub(point,objectP))))
                local raw=add(mul(dx,S.vacuum.stiffness),mul(relV,S.vacuum.damping))
                local fn
                if dn<0 then fn=clamp(S.vacuum.contactStiffness*dn+S.vacuum.contactDamping*dot(relV,n),-S.vacuum.contactForceCap,0)
                else fn=clamp(dot(raw,n),0,cap)end
                state.maxTensileForce=math.max(state.maxTensileForce or 0,math.max(0,fn));state.maxCompressiveForce=math.max(state.maxCompressiveForce or 0,math.max(0,-fn))
                local ft=sub(raw,mul(n,dot(raw,n)))
                if norm(ft)>S.vacuum.friction*cap then ft=mul(ft,S.vacuum.friction*cap/math.max(norm(ft),1e-12))end
                local force=add(mul(n,fn),ft);F=add(F,force);T=add(T,cross(sub(point,objectP),force));maxForce=math.max(maxForce,norm(force))
                count=count+1;c.pressure=ramp*S.vacuum.pressurePa
                sim.setShapeColor(c.handle,'@compound',sim.colorcomponent_ambient_diffuse,{.12,.35+.4*ramp,.22})
                if c.ventStart and ramp==0 then c.anchor=nil;c.vented=true;event('vented',{cup=i})end
            end
        end
    end
    if not lastSupportCheck or sim.getSimulationTime()-lastSupportCheck>=.05 then
        lastSupportCheck=sim.getSimulationTime();supported=false
        local height=job.kind=='sphere' and .025 or (job.kind=='cylinder' and .04 or (job.kind=='irregular' and S.irregular.halfHeight or (job.kind=='extreme' and S.extreme.halfHeight or .0225)))
        local surfaces={H.floor};for _,h in ipairs(H.supports)do surfaces[#surfaces+1]=h end
        for _,surface in ipairs(surfaces)do
            if sim.getObjectInt32Param(surface,sim.shapeintparam_respondable)==1 then
                local gap,data=distance(job.objectHandle,surface)
                if data and gap<=.0002 and objectP[3]-data[6]>=height*.8 then supported=true;break end
            end
        end
    end
    local rolling={0,0,0};local speed=math.sqrt(w[1]^2+w[2]^2)
    if supported and speed>1e-7 then
        local radius=job.kind=='box' and .0225 or .025
        local inertia=job.kind=='sphere' and .4*S.mass*radius^2 or (job.kind=='cylinder' and S.mass*(3*radius^2+.08^2)/12 or S.mass*.045^2/6)
        local torque=math.min(S.rollingResistance*S.mass*9.81*radius,inertia*speed/math.max(dt,1e-6))
        rolling={-torque*w[1]/speed,-torque*w[2]/speed,0};T=add(T,rolling)
    end
    state.supportResistanceTorque=rolling
    if norm(F)>1e-8 or norm(T)>1e-10 then sim.addForceAndTorque(job.objectHandle,F,T)end
    state.seals=count;state.force=F;state.maxCupForce=math.max(state.maxCupForce or 0,maxForce)
    state.verticalCapacity=capacity
    state.maxSeals=math.max(state.maxSeals or 0,count)
end
local function wrap(dt)
    if lastWrap and sim.getSimulationTime()-lastWrap<.10 then return end
    dt=lastWrap and sim.getSimulationTime()-lastWrap or .10;lastWrap=sim.getSimulationTime()
    for f,g in ipairs(H.digits)do
        local k=g.active
        if k<=#g.joints then
            local q=sim.getJointPosition(g.joints[k]);local sign=f==5 and -1 or 1
            local proposal=q+sign*.24*dt
            local limit=f==5 and -1.59 or 1.40
            local gap=distance(g.links[k],job.objectHandle)
            if gap<=S.contactGap then
                event('contact_hold',{digit=f,segment=k,gap=gap,q=q});g.active=k+1
            elseif sign*(proposal-limit)>0 then
                event('joint_limit',{digit=f,segment=k});g.active=k+1
            else
                sim.setJointPosition(g.joints[k],proposal)
                local minGap=math.huge;local touched=k
                for j=k,#g.links do local d=distance(g.links[j],job.objectHandle);if d<minGap then minGap=d;touched=j end end
                if minGap<S.contactGap*.5 or not digitGroundSafe(g,k) then
                    local lo=q;local hi=proposal
                    for iteration=1,8 do
                        local mid=(lo+hi)/2;sim.setJointPosition(g.joints[k],mid)
                        local dd=math.huge;for j=k,#g.links do dd=math.min(dd,(distance(g.links[j],job.objectHandle)))end
                        if dd<S.contactGap or not digitGroundSafe(g,k)then hi=mid else lo=mid end
                    end
                    sim.setJointPosition(g.joints[k],lo);event('contact_limited',{digit=f,joint=k,touchedSegment=touched,gap=minGap});g.active=k+1
                end
            end
        end
    end
    sealCups()
    local complete=true;for _,g in ipairs(H.digits)do if g.active<=#g.joints then complete=false end end
    if complete and not state.wrapCompleteTime then state.wrapCompleteTime=sim.getSimulationTime()end
    if complete and sim.getSimulationTime()-(state.wrapCompleteTime or sim.getSimulationTime())>3.5 then
        state.cupDiagnostics={};state.graspJoints={}
        for f,g in ipairs(H.digits)do state.graspJoints[f]={};for k,j in ipairs(g.joints)do state.graspJoints[f][k]=sim.getJointPosition(j)end end
        for i,c in ipairs(C)do
            local gap,data=distance(c.handle,job.objectHandle);local M=sim.getObjectMatrix(c.handle,sim.handle_world);local angle=180
            if data then angle=math.acos(clamp(dot({-M[3],-M[7],-M[11]},objectNormal({data[4],data[5],data[6]})),-1,1))*180/math.pi end
            state.cupDiagnostics[i]={cup=i,digit=c.digit,segment=c.segment,cupGap=gap,hostGap=distance(c.link,job.objectHandle),angleDegrees=angle,sealed=c.anchor~=nil}
        end
        if state.seals>=S.vacuum.minSeals and (state.verticalCapacity or 0)>=1.25*S.mass*9.81 then phase('HOLD_TEST')
        else fail('Only '..state.seals..' valid cup seals; refusing to lift.')end
    elseif sim.getSimulationTime()-state.phaseStart>25 then fail('Wrap timed out; object shape or pose unsupported.')end
end
function sysCall_init()
    root=sim.getObject('/OctoAssist');S=json.decode(sim.readCustomStringData(root,'OctoSpec'))
    H=json.decode(sim.readCustomStringData(root,'OctoHandles'));job=json.decode(sim.readCustomStringData(root,'OctoJob')or'null')
    H=resolveGeometryHandles(S,H);if job then job.objectHandle=H.objects[job.kind]end
    E={};L={};R={};C={};state={phase='IDLE',done=false,failed=false,seals=0,maxSeals=0,maxCupForce=0,maxHeight=0}
    handCollection=sim.createCollection(1);sim.addItemToCollection(handCollection,sim.handle_single,H.palm,0)
    for _,g in ipairs(H.digits)do for _,h in ipairs(g.links)do sim.addItemToCollection(handCollection,sim.handle_single,h,0)end end
    for i,c in ipairs(H.cups)do C[i]=c end
    if job then
        assert(sim.getObjectParent(job.objectHandle)==-1,'Payload must be world independent')
        for f,g in ipairs(H.digits)do g.active=1 end
        state.startObject=sim.getObjectPosition(job.objectHandle,sim.handle_world);beginMove(1,'APPROACH')
        if job.recordPath then
            recorder=sim.createVisionSensor(1+2+4+64+128,{640,480,0,0},{.01,3,math.pi/3,.001,0,0,.16,.18,.22,0,0})
            local eye={1.05,-1.10,.95};local target={.25,.02,.42};local z=sub(target,eye);z=mul(z,1/norm(z))
            local x=cross({0,0,1},z);x=mul(x,1/norm(x));local y=cross(z,x)
            sim.setObjectMatrix(recorder,{x[1],y[1],z[1],eye[1],x[2],y[2],z[2],eye[2],x[3],y[3],z[3],eye[3]},sim.handle_world)
            frameCount=0
            recorderClose=sim.createVisionSensor(1+2+4+64+128,{640,480,0,0},{.005,2,math.pi/3,.001,0,0,.16,.18,.22,0,0})
        end
    end
end
function getStatus()
    state.time=sim.getSimulationTime()
    local p=job and sim.getObjectPosition(job.objectHandle,sim.handle_world)or{0,0,0}
    return {time=state.time,phase=state.phase,seals=state.seals,done=state.done,failed=state.failed,reason=state.reason,
        objectPosition=p,maxLift=job and math.max(0,state.maxHeight-job.source[3])or 0,
        destination=job and job.destination or nil}
end
function getLog()return {events=E,samples=L,frames=R,status=state}end
local function actuationStep()
    if not job then return end
    if state.done then applyVacuum(sim.getSimulationTimeStep());return end
    local dt=sim.getSimulationTimeStep();local now=sim.getSimulationTime()
    if (state.phase=='WRAP' or (state.phase:sub(1,5)=='MOVE_' and state.moveIndex<=2)) and norm(sub(sim.getObjectPosition(job.objectHandle,sim.handle_world),job.source))>.025 then
        fail('Payload moved more than 25 mm before lift; aborting unsupported approach.');return
    end
    if state.phase:sub(1,5)=='MOVE_' then
        local path=job.paths[state.moveIndex];local u=clamp((now-state.phaseStart)/path.duration,0,1);local blend=u^3*(10-15*u+6*u*u)
        local q={};for i=1,6 do q[i]=state.moveFrom[i]+blend*(path.q[i]-state.moveFrom[i])end
        local before=armQ();setArm(q)
        local checkGround=not lastGround or now-lastGround>=.10
        if checkGround then lastGround=now end
        if checkGround and not groundSafe()then setArm(before);fail('Hand/floor clearance failed along arm trajectory.')
        elseif u==1 then
            if state.nextPhase=='APPROACH'then beginMove(2,'WRAP')
            elseif state.nextPhase=='CARRY_LIFT'then phase('LIFT_SETTLE')
            elseif state.nextPhase=='CARRY_ABOVE'then beginMove(5,'SETDOWN')
            elseif state.nextPhase=='DONE'then phase('SETTLE')
            else phase(state.nextPhase)end
        end
    elseif state.phase=='WRAP'then wrap(dt)
    elseif state.phase=='HOLD_TEST'then
        if now-state.phaseStart>1 then
            if state.seals<S.vacuum.minSeals or (state.verticalCapacity or 0)<1.25*S.mass*9.81 then fail('Insufficient seal/force capacity before lift.')else beginMove(3,'CARRY_LIFT')end
        end
    elseif state.phase=='LIFT_SETTLE'then
        if now-state.phaseStart>.8 then
            if sim.getObjectPosition(job.objectHandle,sim.handle_world)[3]<job.source[3]+.05 then fail('Measured payload did not lift; refusing to carry.')
            else adjustCarry();beginMove(4,'CARRY_ABOVE')end
        end
    elseif state.phase=='SETDOWN'then
        -- Lower the grasp, not the payload, until a physical support is reached.
        if not state.setdownPalm then
            state.setdownPalm=sim.getObjectPosition(H.palm,sim.handle_world);state.setdownDepth=0
        end
        local gap=distance(job.objectHandle,H.floor)
        for _,h in ipairs(H.supports or {})do
            if sim.getObjectInt32Param(h,sim.shapeintparam_respondable)~=0 then gap=math.min(gap,(distance(job.objectHandle,h)))end
        end
        if gap<=.00015 and not state.supportDepth then
            state.supportDepth=state.setdownDepth;event('support_contact',{gap=gap,depth=state.setdownDepth})
        end
        if state.supportDepth and (state.setdownDepth>=state.supportDepth+.0008 or state.preloadLimited) then
            if gap>.00025 then fail('Physical support contact lost before release.');return end
            if not state.supportSettle then state.supportSettle=now end
            if now-state.supportSettle>.3 then phase('VENT')end
        else
            state.setdownDepth=state.setdownDepth+.005*dt
            local p=add(state.setdownPalm,{0,0,-state.setdownDepth});local before=armQ();setArm(ik(p))
            if not groundSafe()then
                setArm(before);state.setdownDepth=state.setdownDepth-.005*dt
                if state.supportDepth and gap<=.00025 then
                    state.preloadLimited=true;event('preload_limited_by_clearance',{depth=state.setdownDepth,gap=gap})
                else fail('Support setdown would violate hand/floor clearance.')end
            end
            if state.setdownDepth>.005 then fail('No physical support found within 5 mm; refusing release.')end
        end
    elseif state.phase=='VENT'then
        if not state.releaseDigit then state.releaseDigit=1;state.releaseStage='vent';state.releaseStart=now end
        local f=state.releaseDigit;local g=H.digits[f];local elapsed=now-state.releaseStart
        if state.releaseStage=='vent' then
            for i,c in ipairs(C)do
                if c.digit==f and c.anchor and not c.ventStart and elapsed>=(c.segment-1)*.08 then c.ventStart=now;event('vent_start',{cup=i,digit=f})end
            end
            if elapsed>(#g.joints-1)*.08+S.vacuum.rampSeconds+.3 then
                state.releaseStage='open';state.releaseStart=now;state.releaseFrom={}
                for k,j in ipairs(g.joints)do state.releaseFrom[k]=sim.getJointPosition(j)end
                event('unwrap_digit',{digit=f})
            end
        else
            -- Other digits remain sealed while this digit clears the object.
            local rest=f==5 and S.thumb.rest or S.fingers[f].rest
            for k,j in ipairs(g.joints)do
                local delay=k==1 and 0 or 1.4;local u=clamp((elapsed-delay)/1.3,0,1);local a=u^3*(10-15*u+6*u*u)
                sim.setJointPosition(j,state.releaseFrom[k]*(1-a)+rest[k]*a)
            end
            if elapsed>2.7 then
                if f==5 then beginMove(6,'DONE')
                else state.releaseDigit=f+1;state.releaseStage='vent';state.releaseStart=now end
            end
        end
    elseif state.phase=='SETTLE'then
        if now-state.phaseStart>1.5 then
            state.finalObject=sim.getObjectPosition(job.objectHandle,sim.handle_world);state.finalVelocity=sim.getVelocity(job.objectHandle)
            state.placeError=norm(sub(state.finalObject,job.destination));state.done=true;phase('DONE')
        end
    end
    applyVacuum(dt)
    local p=sim.getObjectPosition(job.objectHandle,sim.handle_world);state.maxHeight=math.max(state.maxHeight,p[3]);state.parent=sim.getObjectParent(job.objectHandle)
    if state.parent~=-1 then fail('Payload parent changed unexpectedly.')end
    if state.phase:sub(1,5)=='MOVE_' and state.moveIndex>=3 and state.moveIndex<=5 and state.seals<2 then fail('Insufficient seals during carry; arm stopped.')end
end
function sysCall_actuation()
    local ok,err=xpcall(actuationStep,debug.traceback)
    if not ok then fail(err)end
end
function sysCall_sensing()
    if not job then return end
    if state.done and doneLog then return end
    if recorder and not state.failed and (not lastFrame or sim.getSimulationTime()-lastFrame>=.10 or (state.done and not doneFrame)) and (not state.done or not doneFrame) then
        lastFrame=sim.getSimulationTime();sim.handleVisionSensor(recorder)
        local img,res=sim.getVisionSensorImg(recorder);frameCount=frameCount+1
        R[#R+1]={index=frameCount,time=lastFrame,phase=state.phase,seals=state.seals}
        sim.saveImage(img,res,0,job.recordPath..string.format('/frame_%05d.png',frameCount),-1)
        local p=sim.getObjectPosition(H.palm,sim.handle_world);local eye=add(p,{.14,-.19,.12});local target=add(p,{0,-.06,-.035})
        local z=sub(target,eye);z=mul(z,1/norm(z));local x=cross({0,0,1},z);x=mul(x,1/norm(x));local y=cross(z,x)
        sim.setObjectMatrix(recorderClose,{x[1],y[1],z[1],eye[1],x[2],y[2],z[2],eye[2],x[3],y[3],z[3],eye[3]},sim.handle_world)
        sim.handleVisionSensor(recorderClose);local closeImg,closeRes=sim.getVisionSensorImg(recorderClose)
        sim.saveImage(closeImg,closeRes,0,job.recordPath..string.format('/close_%05d.png',frameCount),-1)
        if state.done then doneFrame=true end
    end
    if not lastLog or sim.getSimulationTime()-lastLog>=.05 then
        lastLog=sim.getSimulationTime();L[#L+1]={t=lastLog,phase=state.phase,object=sim.getObjectPosition(job.objectHandle,sim.handle_world),palm=sim.getObjectPosition(H.palm,sim.handle_world),seals=state.seals,force=state.force or {0,0,0},parent=sim.getObjectParent(job.objectHandle)}
        if state.done then doneLog=true end
    end
end
function sysCall_cleanup()
    if handCollection then sim.destroyCollection(handCollection)end
end
