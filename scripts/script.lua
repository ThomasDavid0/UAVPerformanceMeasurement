local MODE_AUTO = 10

local current_state = nil
local active_turn = nil


--[[
   Wrapper to construct a Vector3f{x, y, z} from (x, y, z)
--]]
local function makeVector3f(x, y, z)
    local vec = Vector3f()
    vec:x(x)
    vec:y(y)
    vec:z(z)
    return vec
 end
 
--[[
   return a quaternion for a roll, pitch, yaw (321 euler sequence) attitude
--]]
function qorient(roll_deg, pitch_deg, yaw_deg)
    local q = Quaternion()
    q:from_euler(math.rad(roll_deg), math.rad(pitch_deg), math.rad(yaw_deg))
    return q
end

 --[[
    rotate a vector by a quaternion
 --]]
 local function quat_earth_to_body(quat, v)
    local v2 = v:copy()
    quat:earth_to_body(v2)
    return v2
 end
 
 --[[
    rotate a vector by a inverse quaternion
 --]]
 local function quat_body_to_earth(quat, v)
    local v2 = v:copy()
    quat:inverse():earth_to_body(v2)
    return v2
 end
 
 --[[
    copy a quaternion
 --]]
 local function quat_copy(q)
    return q:inverse():inverse()
 end
 


local function gcsWrite(text)
    gcs:send_text(6, string.format("LUA: %s", text))
end

-- constrain a value between limits
local function constrain(v, vmin, vmax)
    if v < vmin then
       v = vmin
    end
    if v > vmax then
       v = vmax
    end
    return v
 end

-- a PI controller implemented as a Lua object
local function PI_controller(kP,kI,iMax,min,max)
    -- the new instance. You can put public variables inside this self
    -- declaration if you want to
    local self = {}

    -- private fields as locals
    local _kP = kP or 0.0
    local _kI = kI or 0.0
    local _iMax = iMax
    local _min = min
    local _max = max
    local _last_t = nil
    local _I = 0
    local _P = 0
    local _total = 0
    local _counter = 0
    local _target = 0
    local _current = 0

    -- update the controller.
    function self.update(target, current)
        local now = millis():tofloat() * 0.001
        if not _last_t then
            _last_t = now
        end
        local dt = now - _last_t
        _last_t = now
        local err = target - current
        _counter = _counter + 1

        local P = _kP * err
        if ((_total < _max and _total > _min) or (_total >= _max and err < 0) or (_total <= _min and err > 0)) then
            _I = _I + _kI * err * dt
        end
        if _iMax then
            _I = constrain(_I, -_iMax, iMax)
        end
        local I = _I
        local ret = P + I

        _target = target
        _current = current
        _P = P

        ret = constrain(ret, _min, _max)
        _total = ret
        return ret
    end

    function self.reset(integrator)
        _I = integrator
    end
    function self.set_I(I)
        _kI = I
    end
    function self.set_P(P)
        _kP = P
    end
    function self.set_Imax(Imax)
        _iMax = Imax
    end

    return self
end


local function Turn(id, g, alt, arspd)
    local self = {}
    local _id = id
    local _g = g
    local _alt = alt
    local _stage = 0
    local _arspd = arspd
    local _roll_angle = 0

    function self.arspd()
        return _arspd
    end
    function self.id()
        return _id
    end
    function self.g()
        return _g
    end
    function self.alt()
        return _alt
    end
    function self.stage()
        return _stage
    end

    function self.next_stage()
        _stage = _stage + 1
    end

    function self.summary()
        return string.format("Turn: %i, target g: %f", _id, _g)
    end


    return self
end

local function State()
    local self = {}
    local _pos = ahrs:get_relative_position_NED_origin()
    local _att = ahrs:get_quaternion()
    local _arspd = ahrs:airspeed_estimate()
    local _acc = ahrs:get_accel()
    local _roll_angle =  math.asin(quat_body_to_earth(_att, makeVector3f(0,1,0)):z())
    function self.pos()
        return _pos
    end
    function self.att()
        return _att
    end
    function self.arspd()
        return _arspd
    end
    function self.acc()
        return _acc
    end

    function self.roll_angle()
        return _roll_angle
    end

    return self
end


local function initialise(id, cmd)
    if cmd == 1 then
        active_turn = Turn(id, 1.5, ahrs:get_relative_position_NED_origin():z(), ahrs:airspeed_estimate())
        
        gcsWrite(active_turn:summary())

    end
end


local function timout()
    if active_turn then
        gcsWrite(string.format("timeout %s",active_turn:summary()))
        active_turn = nil
    end
end


local speed_pi = PI_controller(5, 25, 100, 0, 100)

local roll_angle_pi = PI_controller(5, 10, 45, -90, 90)
local pitch_alt_controller = PI_controller(5, 10, 40, -40, 40)
local roll_alt_controller = PI_controller(10, 15, 40, -40, 40)

function update()

    if vehicle:get_mode() == MODE_AUTO then
        
        id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()

        if cmd then
            current_state = State()
            if active_turn then
                
                local current_roll_angle = current_state:roll_angle()  * 180 / math.pi
                local target_roll_angle = 0.0

                gcsWrite(string.format("roll angle: %f", current_state:roll_angle() * 180 / math.pi))

                local roll = roll_angle_pi:update(target_roll_angle, current_state:roll_angle() * 180 / math.pi)
                
                local thr = speed_pi.update(active_turn:arspd(), current_state:arspd())
                

                local pitch = 0.0
                local yaw = 0.0 -- perhaps just use this to minimize sideslip
                
                if -current_state:acc():z() > active_turn:g() * 9.81 and active_turn:stage()==0 then
                    -- stage 0 is complete when target g force is reached
                    --active_turn:next_stage()
                    --gcsWrite(string.format("moving to stage %i", active_turn:stage()))
                end

                if active_turn:stage() == 0 then
                    -- Initially roll at constant rate, control altitude with pitch. 
                    --roll = roll_angle_controller:update(target_roll_angle, current_roll_angle)
                    --pitch = -pitch_alt_controller.update(active_turn:alt(), current_state:pos():z())

                elseif active_turn:stage() == 1 then
                    -- maintain g force with elevator, control altitude with roll angle
                    pitch = 10.0
                end
                vehicle:set_target_throttle_rate_rpy(thr, roll, pitch, 0)
                
                logger:write(
                    'TRN1','id,cmd,stage,g,alt,despitch,arspd,thr,roll,desroll','iiifffffff', 
                    id, cmd, active_turn:stage(), 
                    current_state:acc():z(), current_state:pos():z(), 
                    pitch, current_state:arspd(), thr, current_state:roll_angle(), target_roll_angle
                )

            else
                initialise(id, cmd)
            end
        else
            timout()
        end
        
        

    end

    return update, 1000.0/40
end


return update()