local loaded_files = {}
local files = {}
local require = function(name)
    if loaded_files[name] == nil then
        loaded_files[name] = files[name]()
    end
    return loaded_files[name]
end
files['controller'] = function(...)
    
    local controller = {}
    
    -- constrain a value between limits
    function controller.constrain(v, vmin, vmax)
        if v < vmin then
           v = vmin
        end
        if v > vmax then
           v = vmax
        end
        return v
     end
    
    
    
    function controller.PID(name,kP,kI,kD,min,max)
       local self = {}
       local _name = name
       local _kP = kP or 0.0
       local _kI = kI or 0.0
       local _kD = kD or 0.0
       
       local _min = min
       local _max = max
       
       local _P = 0
       local _I = (_min + _max) / 2
       local _D = 0
    
       local _t = nil
       local _err = nil
       local _total = (_min + _max) / 2
    
       function self.update(target, current)
          local t = millis():tofloat() * 0.001
          if not _t then
             _t = t
          end
          local err = target - current
          local dt = t - _t
    
          _P = _kP * err
          if ((_total < _max and _total > _min) or (_total >= _max and err < 0) or (_total <= _min and err > 0)) then
             _I = controller.constrain(_I + _kI * err * dt, _min, _max)
          end
          if dt > 0 then
             _D = _kD * (err - _err) / dt
          end
          
          _t = t
          _err = err
          _total = controller.constrain(_P + _I + _D, _min, _max)
          
          logger.write(
             _name,'Targ,Curr,err,dt,P,I,D,Total','ffffffff',
             target,current,err,dt,_P,_I,_D,_total
          )
    
          return _total
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
    
       function self.set_D(D)
          _kD = D
       end
       
       
       -- return the instance
       return self
    end
    
    return controller
end
files['geometry'] = function(...)
    local g = {}
    
    function g.makeVector3f(x, y, z)
        local vec = Vector3f()
        vec:x(x)
        vec:y(y)
        vec:z(z)
        return vec
    end
    
    function g.vec_mul(v, s)
        return g.makeVector3f(v:x() * s, v:y() * s, v:z() * s)
    end
    
    function g.makeQuaternion(w, x, y, z)
        local q = Quaternion()
        q:q1(w)
        q:q2(x)
        q:q3(y)  
        q:q4(z)
        return q
    end
    
    function g.quat_log(name, q)
        logger.write(
            name, 'w,x,y,z', 'ffff',
            q:q1(), q:q2(), q:q3(), q:q4()
        )
    end
    
    function g.qorient(roll, pitch, yaw)
        local q = Quaternion()
        q:from_euler(roll, pitch, yaw)
        return q
    end
    
    
    function g.qorient_deg(roll_deg, pitch_deg, yaw_deg)
        local q = Quaternion()
        q:from_euler(math.rad(roll_deg), math.rad(pitch_deg), math.rad(yaw_deg))
        return q
    end
    
    function g.quat_earth_to_body(quat, v)
        local v2 = v:copy()
        quat:earth_to_body(v2)
        return v2
    end
    
    function g.quat_body_to_earth(quat, v)
        local v2 = v:copy()
        quat:inverse():earth_to_body(v2)
        return v2
    end
    
    function g.quat_copy(q)
        return q:inverse():inverse()
    end
    
    function g.angle_between(v1, v2)
        return v1:angle(v2)
    end
    
    
    function g.quat_axis(q)
        local q2 = g.quat_copy(q)
        return g.makeVector3f(q2:q2(),q2:q3(),q2:q4())
    end
    
    function g.quat_conjugate(q)
        local q2 = g.quat_copy(q)
        return g.makeQuaternion(q2:q1(), -q2:q2(), -q2:q3(), -q2:q4())
    end
    
    function g.quat_mul(a, b)
        local w = a:q1() * b:q1() - g.quat_axis(a):dot(g.quat_axis(b))
        local xyz = g.vec_mul(g.quat_axis(b), a:q1())  + g.vec_mul(g.quat_axis(a), b:q1()) + g.quat_axis(a):cross(g.quat_axis(b))
        return g.makeQuaternion(w, xyz:x(), xyz:y(), xyz:z())
    end
    
    function g.body_axis_rates(a,b)
        local q = g.quat_mul(g.quat_conjugate(a), b)
        q:normalize()
        local vo = Vector3f()
        q:to_axis_angle(vo)
        return vo
    end
    
    --[[
       get quaternion rotation between vector1 and vector2
       with thanks to https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
    --]]
    function g.vectors_to_quat_rotation(vector1, vector2)
        local v1 = vector1:copy()
        local v2 = vector2:copy()
        v1:normalize()
        v2:normalize()
        local dot = v1:dot(v2)
        local a = v1:cross(v2)
        local w = 1.0 + dot
        local q = Quaternion()
        q:q1(w)
        q:q2(a:x())
        q:q3(a:y())
        q:q4(a:z())
        q:normalize()
        return q
     end
    
     -- convert a quaternion to axis angle form
    function g.to_axis_and_angle(quat)
        local axis_angle = Vector3f()
        quat:to_axis_angle(axis_angle)
        local angle = axis_angle:length()
        if angle < 0.00001 then
           return g.makeVector3f(1.0, 0.0, 0.0), 0.0
        end
        return axis_angle:scale(1.0/angle), angle
     end
    
    
     return g
end
files['state'] = function(...)
    local g = require('geometry')
    local c = require('controller')
    
    local State = {}
    
    function State.new(pos, att, arspd, vel, acc, wind)
        local self = {}
        local _pos = pos
        local _att = att
        local _arspd = arspd
        local _vel = vel
        local _acc = acc
        local _wind = wind
    
        local _hv = g.quat_earth_to_body(_att, g.makeVector3f(1,0,0))
        local _yaw = math.atan(_hv:y(), _hv:x())
        local _pitch = -math.atan(_hv:z(), math.sqrt(_hv:x() * _hv:x() + _hv:y() * _hv:y()))
        local _roll_angle = g.body_axis_rates(g.qorient(0, _pitch, _yaw), _att):x()
        
        function self.pos()
            return _pos
        end
        function self.att()
            return _att
        end
        function self.arspd()
            return _arspd
        end
        function self.vel()
            return _vel
        end
        function self.acc()
            return _acc
        end
        function self.wind()
            return _wind
        end
        function self.roll_angle()
            return _roll_angle
        end
        
        function self.log(name)
            logger.write(
                name, 'arspd,roll,pitch,yaw', 'ffff',
                _arspd, _roll_angle * 180 / math.pi, _pitch * 180/math.pi , _yaw * 180 / math.pi
            )
        end
        
        return self
    end
    
    function State.readCurrent()
        return State.new(
            ahrs:get_relative_position_NED_origin(),
            ahrs:get_quaternion(),
            c.constrain(ahrs:get_EAS2TAS() * math.max(ahrs:airspeed_estimate(), 3), 3, 100),
            ahrs:get_velocity_NED(),
            ahrs:get_accel(),
            ahrs:wind_estimate()
        )
    end
    
    
    
    return State
end
files['comms'] = function(...)
    
    
    local comms = {}
    
    function comms.gcsWrite(text)
        gcs:send_text(6, string.format("LUA: %s", text))
    end
    
    
    return comms

end
files['turn'] = function(...)
    
    local comms = require('comms')
    
    local turn = {}
    
    
    function turn.Turn(id, load_factor, alt, arspd)
        local self = {}
        local _id = id
        local _load_factor = load_factor
        local _alt = alt
        local _stage = 0
        local _arspd = arspd
        local _roll_angle = math.acos(1 / _load_factor)
    
        function self.arspd()
            return _arspd
        end
        function self.id()
            return _id
        end
        function self.load_factor()
            return _load_factor
        end
        function self.alt()
            return _alt
        end
        function self.stage()
            return _stage
        end
        function self.stagestring()
            return string.format('STG%i',_stage)
        end
        function self.next_stage()
            _stage = _stage + 1
        end
        function self.roll_angle()
            return _roll_angle
        end
    
        function self.summary()
            return string.format("Turn: %i, target g: %f", _id, _load_factor)
        end  
    
        return self
    end
    
    
    function turn.initialise(id, cmd)
        if cmd == 1 then
            local new_turn = turn.Turn(id, 2.0, ahrs:get_relative_position_NED_origin():z(), ahrs:airspeed_estimate())
            comms.gcsWrite(new_turn:summary())
            return new_turn
        end
    end
    
    function turn.timout(active_turn)
        if active_turn then
            comms.gcsWrite(string.format("timeout %s",active_turn.summary()))
            return nil
        end
    end
    
    
    return turn
end

local controller = require('controller')
local g = require('geometry')
local State = require('state')
local t = require('turn')

local MODE_AUTO = 10


local active_turn = nil

local speed_controller = controller.PID('TSPD', 13, 2, 0.0, 10, 100)
local roll_controller = controller.PID('TRLL', 2.3, 0.0, 0.14, -360, 360)

function update()

    if vehicle:get_mode() == MODE_AUTO and arming:is_armed() then
        
        local id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()
        
        if cmd then
            if active_turn then
                local current_state = State.readCurrent()

                local thr = speed_controller.update(active_turn:arspd(), current_state.arspd())

                local roll = roll_controller.update(active_turn.roll_angle()*180/math.pi, current_state.roll_angle() * 180 / math.pi)
                
                local w_z_err = (active_turn.alt() - current_state.pos():z()) / math.cos(current_state.roll_angle())
                local b_z_err = g.quat_earth_to_body(current_state.att(), g.makeVector3f(0,0,w_z_err)):z()
               
                local pitch = -180 * (b_z_err * 2 - current_state.vel():z() * 2.8) / (current_state.arspd() * math.pi)

                local yaw = 0.0

                logger.write(
                    'TINF', 'id,cmd,loadfactor', 'iif',
                    id, cmd, active_turn.load_factor()
                )

                vehicle:set_target_throttle_rate_rpy(thr, roll, pitch, yaw)
                

            else
                active_turn = t.initialise(id, cmd)
            end
        else
            active_turn = nil
        end
    end
    return update, 1000.0/40
end


return update()


