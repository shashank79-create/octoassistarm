-- Bounded RPC work: one sampled configuration per call, never a Lua coroutine.
local sim=require('sim');local json=require('dkjson');local D
local function restore()
    if not D or D.restored then return end
    if D.collection then pcall(sim.destroyCollection,D.collection)end
    if D.clones then
        local ok,err=pcall(sim.removeObjects,D.clones)
        if not ok then D.fault=true;D.reason='Preflight clone cleanup failed: '..tostring(err)end
    end
    D.restored=true
end
local function result()
    return {done=D.completed,passed=D.completed and not D.fault and not D.badStage,
        minimumGround=D.minimum<math.huge and D.minimum or -1,
        minimumApproachObject=D.minObject<math.huge and D.minObject or -1,
        samples=D.samples,badStage=D.badStage,reason=D.reason or ''}
end
local function guarded(fn)
    -- Remote functions may run outside a coroutine. Suppress automatic yields
    -- only for this short call, then restore the script's previous setting.
    local previous=sim.setStepping(true)
    local ok,value=xpcall(fn,debug.traceback)
    if not ok then
        D=D or {oldQ={},oldF={},samples=0,minimum=math.huge,minObject=math.huge}
        D.fault=true;D.completed=true;D.reason='Preflight Lua error: '..tostring(value)
        restore();value=result()
    end
    sim.setStepping(previous)
    return value
end
function beginPaths(spec,handles,paths,objectHandle)
    return guarded(function()
        restore()
        D={oldQ={},oldF={},bodies={},samples=0,minimum=math.huge,
           minObject=math.huge,completed=false,reason='',object=objectHandle}
        D.s=json.decode(spec);D.H=json.decode(handles);D.P=json.decode(paths)
        assert(type(D.s)=='table' and type(D.H)=='table' and type(D.P)=='table','Invalid preflight JSON')
        assert(#D.P==6,'Preflight requires six arm paths')
        -- Check an invisible, nonrespondable copy. Never pose the real robot.
        local originalH=json.decode(handles)
        local originals=sim.getObjectsInTree(originalH.arm[1],sim.handle_all,0)
        D.clones=sim.copyPasteObjects(originals,2+4+8+32)
        local map={}
        for i,h in ipairs(originals)do
            local c=D.clones[i];assert(c and c>=0,'Unable to copy preflight geometry')
            map[h]=c;sim.setObjectAlias(c,'__OctoPreflight_'..c)
            sim.setObjectInt32Param(c,sim.objintparam_visibility_layer,0)
            if sim.getObjectType(c)==sim.object_shape_type then
                sim.setObjectInt32Param(c,sim.shapeintparam_respondable,0)
                sim.setObjectInt32Param(c,sim.shapeintparam_static,1)
            end
        end
        sim.setObjectParent(map[originalH.arm[1]],sim.handle_world,true)
        D.H.palm=assert(map[originalH.palm],'Palm missing from preflight copy')
        for i,h in ipairs(originalH.arm)do D.H.arm[i]=assert(map[h])end
        for f,g in ipairs(originalH.digits)do
            for k,h in ipairs(g.joints)do D.H.digits[f].joints[k]=assert(map[h])end
            for k,h in ipairs(g.links)do D.H.digits[f].links[k]=assert(map[h])end
        end
        D.collection=sim.createCollection(1)
        for _,h in ipairs(sim.getObjectsInTree(originalH.palm,sim.object_shape_type,0))do
            if sim.getObjectInt32Param(h,sim.shapeintparam_respondable)==1 then
                local c=assert(map[h]);D.bodies[#D.bodies+1]=c;sim.addItemToCollection(D.collection,sim.handle_single,c,0)
            end
        end
        assert(#D.bodies>0,'Preflight hand collection is empty')
        for i,j in ipairs(D.H.arm)do D.oldQ[i]=sim.getJointPosition(j)end
        for f,g in ipairs(D.H.digits)do
            D.oldF[f]={};for k,j in ipairs(g.joints)do D.oldF[f][k]=sim.getJointPosition(j)end
        end
        return result()
    end)
end
function nextPathSample()
    return guarded(function()
        assert(D,'Call beginPaths before requesting a sample')
        if D.completed then return result()end
        local stage=math.floor(D.samples/21)+1;local step=D.samples%21
        local s,H=D.s,D.H;local path=D.P[stage]
        if step==0 then
            for f,g in ipairs(H.digits)do
                local angles
                if stage>=3 and stage<=5 then angles=f==5 and s.thumb.nominal or s.fingers[f].nominal
                else angles=f==5 and s.thumb.rest or s.fingers[f].rest end
                for k,j in ipairs(g.joints)do sim.setJointPosition(j,angles[k])end
            end
        end
        local start=stage==1 and D.oldQ or D.P[stage-1].q;local u=step/20
        for i,j in ipairs(H.arm)do sim.setJointPosition(j,start[i]+u*(path.q[i]-start[i]))end
        local obstacles={H.floor};for _,v in ipairs(H.supports)do obstacles[#obstacles+1]=v end
        for _,obstacle in ipairs(obstacles)do
            local ok,data=sim.checkDistance(D.collection,obstacle,0)
            assert(ok==1,'Distance query failed for '..sim.getObjectAlias(obstacle)..' at stage '..stage)
            D.minimum=math.min(D.minimum,data[7])
            if data[7]<s.groundMargin then D.badStage=stage;D.reason='Hand/support clearance failed.'end
        end
        if stage<=2 and D.object then
            local ok,data=sim.checkDistance(D.collection,D.object,0)
            assert(ok==1,'Payload approach distance query failed')
            D.minObject=math.min(D.minObject,data[7])
            if data[7]<.0001 then
                D.badStage=stage;D.reason='Open hand intersects the payload during approach; shape/pose unsupported.'
                for _,body in ipairs(D.bodies)do
                    local hit,d=sim.checkDistance(body,D.object,0)
                    if hit==1 and d[7]<.0001 then D.reason=D.reason..' Body: '..sim.getObjectAlias(body);break end
                end
            end
        end
        for _,other in pairs(H.objects or {})do
            if other~=D.object then local ok,data=sim.checkDistance(D.collection,other,0);assert(ok==1,'Other object clearance query failed');if data[7]<.002 then D.badStage=stage;D.reason='Hand path is too close to another scene object.'end end
        end
        D.samples=D.samples+1
        if D.samples==126 then D.completed=true;restore()end
        return result()
    end)
end
function cancelPaths()
    return guarded(function()
        if not D then return {done=true,passed=false,reason='No preflight started',samples=0}end
        if not D.completed then D.completed=true;D.fault=true;D.reason='Preflight cancelled'end
        restore();return result()
    end)
end
function sysCall_cleanup()restore()end
function checkPaths()
    return {passed=false,samples=0,reason='Preflight interface updated. Run clear functions in MATLAB, then run_octopus.'}
end
