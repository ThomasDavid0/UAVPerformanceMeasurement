local loaded_files = {}
local files = {}
local require = function(name)
    if loaded_files[name] == nil then
        loaded_files[name] = files[name]()
    end
    return loaded_files[name]
end
files['pid'] = function(...)
    
    local PID = {}
    
    -- constrain a value between limits
    function PID.constrain(v, vmin, vmax)
        if v < vmin then
           v = vmin
        end
        if v > vmax then
           v = vmax
        end
        return v
     end
    
    
    
    function PID.new(name,kP,kI,kD,min,max)
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
    
       function self:update(target, current)
          local t = millis():tofloat() * 0.001
          if not _t then
             _t = t
          end
          local err = target - current
          local dt = t - _t
    
          _P = _kP * err
          if ((_total < _max and _total > _min) or (_total >= _max and err < 0) or (_total <= _min and err > 0)) then
             _I = PID.constrain(_I + _kI * err * dt, _min, _max)
          end
          if dt > 0 then
             _D = _kD * (err - _err) / dt
          end
          
          _t = t
          _err = err
          _total = PID.constrain(_P + _I + _D, _min, _max)
          
          logger.write(
             _name,'Targ,Curr,err,dt,P,I,D,Total','ffffffff',
             target,current,err,dt,_P,_I,_D,_total
          )
    
          return _total
       end
    
       function self:reset(value)
          if value == nil then
             value = (_min + _max) / 2
          end
          _I = value
          _t=nil
          _err = nil
          _total = value
       end
       function self:I()
          return _I
       end
       function self:P()
          return _P
       end
       function self:D()
          return _D
       end
       function self:total()
          return _total
       end
       
       -- return the instance
       return self
    end
    
    return PID
end
files['geometry/point'] = function(...)
    
    
    local Point = {}
    
    function Point.new(p)
        local self = {}
        local _p = p
    
        function self:x()
            return _p:x()
        end
        function self:y()
            return _p:y()
        end
        function self:z()
            return _p:z()
        end
        function self:copy()
            return Point.xyz(_p:x(), _p:y(), _p:z())
        end
        function self:p()
            return _p
        end
    
        function self:length()
            return _p:length()
        end
    
        function self:scale(value)
            return Point.xyz(_p:x() * value, _p:y() * value, _p:z() * value)
        end
    
        function self:unit()
            return Point.new(self:scale(1/self:length()))
        end
    
        return self
    end
    
    function Point.xyz(x, y, z)
        local vo = Vector3f()
        vo:x(x)
        vo:y(y)
        vo:z(z)
        return Point.new(vo)
    end
    
    function Point.add(p1, p2)
        return Point.xyz(p1:x() + p2:x(), p1:y() + p2:y(), p1:z() + p2:z())
    end
    
    function Point.sub(p1, p2)
        return Point.xyz(p1:x() - p2:x(), p1:y() - p2:y(), p1:z() - p2:z())
    end
    
    function Point.dot(p1, p2)
        return p1:x()*p2:x() + p1:y()*p2:y() + p1:z()*p2:z()
    end
    
    function Point.cross(p1, p2)
        return Point.new(p1:p():cross(p2:p()))
    end
    function Point.x(v)
        return Point.xyz(v, 0, 0)
    end
    function Point.y(v)
        return Point.xyz(0, v, 0)
    end
    function Point.z(v)
        return Point.xyz(0, 0, v)
    end
    
    
    
    return Point
end
files['geometry/quaternion'] = function(...)
    
    local Point = require('geometry/point')
    
    local Quat = {}
    
    
    function Quat.new(q)
        local self = {}
        local _q = q
    
        function self:w()
            return _q:q1()
        end
        function self:x()
            return _q:q2()
        end
        function self:y()
            return _q:q3()
        end
        function self:z()
            return _q:q4()
        end
        function self:q1()
            return _q:q1()
        end
        function self:q2()
            return _q:q2()
        end
        function self:q3()
            return _q:q3()
        end
        function self:q4()
            return _q:q4()
        end
        function self:q()
            return _q
        end
        function self:normalize()
            local qo = self:copy():q()
            qo:normalize()
            return Quat.new(qo)
        end
        function self:copy()
            return Quat.wxyz(_q:q1(), _q:q2(), _q:q3(), _q:q4())
        end
        function self:inverse()
            return Quat.new(_q:inverse())
        end
        function self:transform_point(v)
            local vo = v:copy():p()
            _q:earth_to_body(vo)
            return Point.new(vo)
        end
        function self:axis()
            return Point.xyz(_q:q2(), _q:q3(), _q:q4())
        end
        function self:conjugate()
            return Quat.wxyz(_q:q1(), -_q:q2(), -_q:q3(), -_q:q4())
        end
        function self:to_axis_angle()
            local vo = Vector3f()
            _q:to_axis_angle(vo)
            return Point.new(vo)
        end
        return self
    end
    
    function Quat.wxyz(w, x, y, z)
        local q = Quaternion()
        q:q1(w)
        q:q2(x)
        q:q3(y)
        q:q4(z)
        return Quat.new(q)
    end
    function Quat.euler(p)
        local _q = Quaternion()
        _q:from_euler(p:x(), p:y(), p:z())
        return Quat.new(_q)
    end
    function Quat.euldeg(p)
        return Quat.euler(Point.xyz(math.rad(p:x()), math.rad(p:y()), math.rad(p:z())))
    end
    function Quat.mul(a, b)
        local w = a:w() * b:w() - Point.dot(a:axis(), b:axis())
        local xyz = Point.add(Point.add(b:axis():scale(a:w()), a:axis():scale(b:w())), Point.cross(a:axis(), b:axis()))
    
        return Quat.wxyz(w, xyz:x(), xyz:y(), xyz:z())
    end
    function Quat.body_axis_rates(a, b)
        return Quat.mul(a:conjugate(), b):normalize():to_axis_angle()
    end
    
    return Quat
end
files['state'] = function(...)
    
    local P = require('geometry/point')
    local Q = require('geometry/quaternion')
    local PID = require('pid')
    
    local State = {}
    
    function State.new(pos, att, arspd, vel, acc, wind)
        local self = {}
        local _pos = pos
        local _att = att
        local _arspd = arspd
        local _vel = vel
        local _acc = acc
        local _wind = wind
    
        local _hv = _att:transform_point(P.xyz(1,0,0))
        local _yaw = math.atan(_hv:y(), _hv:x())
        local _pitch = -math.atan(_hv:z(), math.sqrt(_hv:x() * _hv:x() + _hv:y() * _hv:y()))
        local _roll_angle = Q.body_axis_rates(Q.euler(P.xyz(0, _pitch, _yaw)), _att):x()
        
        function self:pos()
            return _pos
        end
        function self:att()
            return _att
        end
        function self:arspd()
            return _arspd
        end
        function self:vel()
            return _vel
        end
        function self:acc()
            return _acc
        end
        function self:wind()
            return _wind
        end
        function self:roll_angle()
            return _roll_angle
        end
        function self:pitch_angle()
            return _pitch
        end
    
    
        function self:log(name)
            logger.write(
                name, 'arspd,roll,pitch,yaw', 'ffff',
                _arspd, _roll_angle * 180 / math.pi, _pitch * 180/math.pi , _yaw * 180 / math.pi
            )
        end
        
        return self
    end
    
    function State.readCurrent()
        local pos = P.new(ahrs:get_relative_position_NED_origin())
        local att = Q.new(ahrs:get_quaternion())
        local atti = att:inverse()
        return State.new(
            pos,
            att,
            PID.constrain(ahrs:get_EAS2TAS() * math.max(ahrs:airspeed_estimate(), 3), 3, 100),
            atti:transform_point(P.new(ahrs:get_velocity_NED())),
            P.new(ahrs:get_accel()),
            P.new(ahrs:wind_estimate())
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
    
    local Turn = {}
    
    
    function Turn.new(id, load_factor, alt, arspd)
        local self = {}
        local _id = id
        local _load_factor = load_factor
        local _alt = alt
        local _stage = 0
        local _arspd = arspd
        local _roll_angle = math.acos(1 / _load_factor)
        local _start_time = millis() / 1000
    
    
        function self:arspd()
            return _arspd
        end
        function self:id()
            return _id
        end
        function self:load_factor()
            return _load_factor
        end
        function self:alt()
            return _alt
        end
        function self:stage()
            return _stage
        end
        function self:stagestring()
            return string.format('STG%i',_stage)
        end
        function self:next_stage()
            _stage = _stage + 1
        end
        function self:roll_angle()
            return _roll_angle
        end
        function self:start_time()
            return _start_time
        end
        function self:summary()
            return string.format("Turn: %i, target g: %f, roll angle: %f", _id, _load_factor, math.deg(_roll_angle))
        end  
    
        return self
    end
    
    
    function Turn.initialise(id, cmd)
        if cmd == 1 then
            local new_turn = Turn.new(id, 1.1 + 0.2*id, ahrs:get_relative_position_NED_origin():z(), ahrs:airspeed_estimate())
            comms.gcsWrite(new_turn:summary())
            return new_turn
        end
    end
    
    function Turn.timout(active_turn)
        if active_turn then
            comms.gcsWrite(string.format("timeout %s",active_turn.summary()))
            return nil
        end
    end
    
    
    return Turn
end

local PID = require('pid')
local P = require('geometry/point')
local State = require('state')
local Turn = require('turn')

local MODE_AUTO = 10

local turn = nil

local speed_controller = PID.new('TSPD', 13, 10, 0.0, 10, 100)
local rollalt_controller = PID.new('TRAL', 0.5, 0.6, 0.5, 0, 90) -- control vertical speed with roll angle
local rollangle_controller = PID.new('TRAN', 0.6, 0.2, 0.2, -90, 90) -- control roll angle with roll rate


local pitch_controller = PID.new('TPIT', 4, 2, 4.5, -180, 180)
local pitch_g_controller = PID.new('TPIG', 3, 10, 1.0, -180, 180)
local yaw_controller = PID.new('TYAW', 1.5, 2.5, 1.0, -20, 20)

local stagetimer = 0

function update()

    if vehicle:get_mode() == MODE_AUTO and arming:is_armed() then
        
        local id, cmd, arg1, arg2, arg3, arg4 = vehicle:nav_script_time()
        
        if cmd then
            if turn then

                local state = State.readCurrent()

                local thr = speed_controller:update(
                    turn:arspd(),
                    state:arspd()
                )
                local yaw = yaw_controller:update(0.0, -state:vel():y())
                local roll = 10.0
                local pitch = 0.0

                if turn:stage() == 0 and turn:roll_angle() < state:roll_angle() then
                    gcs:send_text(6, 'LUA: moving to stage 1')
                    turn:next_stage()
                    rollangle_controller:reset(0)
                    stagetimer = millis() / 1000
                elseif turn:stage() == 1 and (millis() / 1000 - stagetimer > 1)then
                    gcs:send_text(6, 'LUA: moving to stage 2')
                    turn:next_stage()
                    rollalt_controller:reset(math.deg(state:roll_angle()))
                    pitch_g_controller:reset(pitch_controller:I())
                    stagetimer = millis() / 1000
                end

                if turn:stage() < 2 then
                    if turn:stage() == 1 then
                        roll = rollangle_controller:update(math.deg(turn:roll_angle()), math.deg(state:roll_angle()))
                    end
                    local w_z_err = (turn:alt() - state:pos():z()) / math.cos(state:roll_angle())
                    local pitch_err = state:att():transform_point(P.z(w_z_err)):z() / state:vel():length()
                    pitch = pitch_controller:update(0.0,  math.deg(pitch_err))
                elseif turn:stage() == 2 then
                    pitch = pitch_g_controller:update(turn:load_factor(), -state:acc():z() / 9.81 )
                    local roll_angle = rollalt_controller:update(0, state:att():transform_point(state:vel()):z())
                    roll = rollangle_controller:update(roll_angle, math.deg(state:roll_angle()))
                end

                logger.write('TINF', 'id,cmd,loadfactor,stage', 'iifi', id, cmd, turn:load_factor(), turn:stage())

                vehicle:set_target_throttle_rate_rpy(thr, roll, pitch, yaw)
                vehicle:set_rudder_offset(0, true)

            else
                turn = Turn.initialise(id, cmd)
                gcs:send_text(6, string.format('LUA: %s', turn:summary()))
            end
        else
            turn = nil
        end
    end
    return update, 1000.0/40
end


return update()


