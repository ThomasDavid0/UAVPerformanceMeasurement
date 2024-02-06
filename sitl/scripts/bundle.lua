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
files['geometry/point'] = function(...)
    
    
    local Point = {}
    
    function Point.new(p)
        local self = {}
        local _p = p
    
        function self.x()
            return _p:x()
        end
        function self.y()
            return _p:y()
        end
        function self.z()
            return _p:z()
        end
        function self.copy()
            return Point.xyz(self:x(), self:y(), self:z())
        end
        function self.p()
            return _p
        end
    
        function self.length()
            return _p:length()
        end
    
        function self.scale(value)
            return Point.xyz(self.x() * value, self.y() * value, self.z() * value)
        end
    
        function self.unit()
            return Point.new(self.scale(1/self.length()))
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
        return Point.xyz(p1:y()*p2:z() - p1:z()*p2:y(), p1:z()*p2:x() - p1:x()*p2:z(), p1:x()*p2:y() - p1:y()*p2:x())
    end
    
    
    return Point
end
files['geometry/quaternion'] = function(...)
    
    local Point = require('geometry/point')
    
    local Quat = {}
    
    
    function Quat.new(q)
        local self = {}
        local _q = q
    
        function self.w()
            return _q:q1()
        end
        function self.x()
            return _q:q2()
        end
        function self.y()
            return _q:q3()
        end
        function self.z()
            return _q:q4()
        end
        function self.q1()
            return _q:q1()
        end
        function self.q2()
            return _q:q2()
        end
        function self.q3()
            return _q:q3()
        end
        function self.q4()
            return _q:q4()
        end
        function self.q()
            return _q
        end
        function self.copy()
            return Quat.wxyz(self.q1(), self.q2(), self.q3(), self.q4())
        end
        function self.inverse()
            return Quat.new(_q:inverse())
        end
        function self.transform_point(v)
            local vo = v.copy()
            _q:earth_to_body(vo:p())
            return vo
        end
        function self.axis()
            return Point.xyz(self:x(), self:y(), self:z())
        end
        function self.conjugate()
            return Quat.wxyz(self.w(), -self.x(), -self.y(), -self.z())
        end
        function self.to_axis_angle()
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
        _q:from_euler(p.x(), p.y(), p.z())
        return Quat.new(_q)
    end
    function Quat.euldeg(p)
        return Quat.euler(p.xyz(math.rad(p.x()), math.rad(p.y()), math.rad(p.z())))
    end
    function Quat.mul(a, b)
        local w = a.w() * b.w() - Point.dot(a.axis(), b.axis())
        local xyz = b.axis().scale(a.w()) + a.axis().scale(b.w()) + Point.cross(a.axis(), b.axis())
        return Quat.wxyz(w, xyz.x(), xyz.y(), xyz.z())
    end
    function Quat.body_axis_rates(a, b)
        return Quat.mul(a.conjugate(), b).normalize().to_axis_angle()
    end
    
    return Quat
end
files['state'] = function(...)
    
    local P = require('geometry/point')
    local Q = require('geometry/quaternion')
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
    
        local _hv = _att.transform_point(P.xyz(1,0,0))
        local _yaw = math.atan(_hv:y(), _hv:x())
        local _pitch = -math.atan(_hv:z(), math.sqrt(_hv:x() * _hv:x() + _hv:y() * _hv:y()))
        local _roll_angle = Q.body_axis_rates(Q.euler(P.xyz(0, _pitch, _yaw)), _att):x()
        
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
            P.new(ahrs:get_relative_position_NED_origin()),
            Q.new(ahrs:get_quaternion()),
            c.constrain(ahrs:get_EAS2TAS() * math.max(ahrs:airspeed_estimate(), 3), 3, 100),
            P.new(ahrs:get_velocity_NED()),
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
local P = require('geometry/point')
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

                local thr = speed_controller.update(active_turn:arspd(), current_state:arspd())

                local roll = roll_controller.update(active_turn.roll_angle()*180/math.pi, current_state.roll_angle() * 180 / math.pi)
                
                local w_z_err = (active_turn.alt() - current_state.pos():z()) / math.cos(current_state.roll_angle())
                local b_z_err = current_state.att().transform_point(P.xyz(0,0,w_z_err)):z()
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


